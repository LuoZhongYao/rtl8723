#include    <linux/kernel.h>
#include    <linux/slab.h>
#include    <linux/suspend.h>
#include    <linux/types.h>
#include    <linux/version.h>
#include    <linux/firmware.h>
#include    <net/bluetooth/bluetooth.h>
#include    <net/bluetooth/hci_core.h>
#include    <net/bluetooth/hci.h>
#include    "patch.h"
#include    "zdebug.h"


#define BTUSB_RPM  0

/*******************************
**    Reasil patch code
********************************/
#define CMD_CMP_EVT		0x0e
#define PKT_LEN			300
#define MSG_TO			1000
#define PATCH_SEG_MAX	252
#define DATA_END		0x80
#define DOWNLOAD_OPCODE	0xfc20
#define BTOFF_OPCODE	0xfc28
#define TRUE			1
#define FALSE			0
#define CMD_HDR_LEN		sizeof(struct hci_command_hdr)
#define EVT_HDR_LEN		sizeof(struct hci_event_hdr)
#define CMD_CMP_LEN		sizeof(struct hci_ev_cmd_complete)

#define HCI_CMD_READ_BD_ADDR 0x1009
#define HCI_VENDOR_CHANGE_BDRATE 0xfc17
#define HCI_VENDOR_READ_RTK_ROM_VERISION 0xfc6d
#define HCI_VENDOR_READ_LMP_VERISION 0x1001

#define ROM_LMP_8723a               0x1200
#define ROM_LMP_8723b               0x8723       
#define ROM_LMP_8821a               0X8821
#define ROM_LMP_8761a               0X8761 

//signature: Realtech
const uint8_t  RTK_EPATCH_SIGNATURE[8]={0x52,0x65,0x61,0x6C,0x74,0x65,0x63,
0x68};
//Extension Section IGNATURE:0x77FD0451
const uint8_t Extension_Section_SIGNATURE[4]={0x51,0x04,0xFD,0x77};
uint16_t project_id[]=
{
	ROM_LMP_8723a,
	ROM_LMP_8723b,
	ROM_LMP_8821a,
	ROM_LMP_8761a
};
struct rtk_eversion_evt {
	uint8_t status;
	uint8_t version;
}__attribute__ ((packed));

struct rtk_epatch_entry{
	uint16_t chipID;
	uint16_t patch_length;
	uint32_t start_offset;
} __attribute__ ((packed));

struct rtk_epatch{
    	uint8_t signature[8];
	uint32_t fm_version;
	uint16_t number_of_total_patch;
	struct rtk_epatch_entry entry[0];
} __attribute__ ((packed));

struct rtk_extension_entry{
	uint8_t opcode;
	uint8_t length;
	uint8_t *data;
} __attribute__ ((packed));
/* Realtek - For rtk_btusb driver end */

uint8_t gEVersion = 0;


enum rtk_endpoit {
	CTRL_EP = 0,
	INTR_EP = 1,
	BULK_EP = 2,
	ISOC_EP = 3
};

typedef struct {
	uint16_t	prod_id;
	uint16_t	lmp_sub;
	char		*patch_name;
	char		*config_name;
	uint8_t		*fw_cache;
	int			fw_len;
} patch_info;

typedef struct {
	struct list_head		list_node;
	struct usb_interface	*intf;
	struct usb_device		*udev;
	struct notifier_block	pm_notifier;
	patch_info				*patch_entry;
} dev_data;

typedef struct {
	dev_data	*dev_entry;
	int			pipe_in, pipe_out;
	uint8_t		*send_pkt;
	uint8_t		*rcv_pkt;
	struct hci_command_hdr		*cmd_hdr;
	struct hci_event_hdr		*evt_hdr;
	struct hci_ev_cmd_complete	*cmd_cmp;
	uint8_t		*req_para, *rsp_para;
	uint8_t		*fw_data;
	int			pkt_len, fw_len;
} xchange_data;

typedef struct {
	uint8_t index;
	uint8_t data[PATCH_SEG_MAX];
} __attribute__((packed)) download_cp;

typedef struct {
	uint8_t status;
	uint8_t index;
} __attribute__((packed)) download_rp;


static dev_data* dev_data_find(struct usb_interface* intf);
static patch_info* get_patch_entry(struct usb_device* udev);
static int rtkbt_pm_notify(struct notifier_block* notifier, ulong pm_event, void* unused);
static int load_firmware(dev_data* dev_entry, uint8_t** buff);
static void init_xdata(xchange_data* xdata, dev_data* dev_entry);
static int check_fw_version(xchange_data* xdata);
static int get_firmware(xchange_data* xdata);
static int download_data(xchange_data* xdata);
static int send_hci_cmd(xchange_data* xdata);
static int rcv_hci_evt(xchange_data* xdata);
static uint8_t rtk_get_eversion(dev_data * dev_entry);
static uint8_t global_eversion = 0xff;


static patch_info patch_table[] = {
    { 0xA761, 0x8761, "rtl8761au_fw", "rtl8761a_config", NULL, 0 }, //Rtl8761AU only
    { 0x818B, 0x8761, "rtl8761aw8192eu_fw", "rtl8761a_config", NULL, 0 }, //Rtl8761Aw + 8192EU
    { 0x818C, 0x8761, "rtl8761aw8192eu_fw", "rtl8761a_config", NULL, 0 }, //Rtl8761Aw + 8192EU
    { 0x8760, 0x8761, "rtl8761au8192ee_fw", "rtl8761a_config", NULL, 0 }, //Rtl8761AU + 8192EE
    { 0xB761, 0x8761, "rtl8761au8192ee_fw", "rtl8761a_config", NULL, 0 }, //Rtl8761AU + 8192EE
    { 0x8761, 0x8761, "rtl8761au8192ee_fw", "rtl8761a_config", NULL, 0 }, //Rtl8761AU + 8192EE for LI
    { 0x8A60, 0x8761, "rtl8761au8812ae_fw", "rtl8761a_config", NULL, 0 }, //Rtl8761AU + 8812AE
    
    { 0x8821, 0x8821, "rtl8821a_fw", "rtl8821a_config", NULL, 0 },  //Rtl8821AE
    { 0x0821, 0x8821, "rtll8821a_fw", "rtl8821a_config", NULL, 0 },  //Rtl8821AU
    
    { 0xb720, 0x8723, "rtl8723b_fw", "rtl8723b_config", NULL, 0 },  //Rtl8723BU
    { 0xb72A, 0x8723, "rtl8723b_fw", "rtl8723b_config", NULL, 0 },  //Rtl8723BU
    { 0xb728, 0x8723, "rtl8723b_fw", "rtl8723b_config", NULL, 0 },  //Rtl8723BE for LC
    { 0xb723, 0x8723, "rtl8723b_fw", "rtl8723b_config", NULL, 0 },  //Rtl8723BE
    
    { 0, 0x1200, "rtl8723a_fw", "rtl8723a_config", NULL, 0 } //Rtl8723AU & Rtl8723AE by default
};

static LIST_HEAD(dev_data_list);

int patch_add(struct usb_interface* intf)
{
	dev_data	*dev_entry;
	struct usb_device *udev;

	message("patch_add");
	dev_entry = dev_data_find(intf);
	if (NULL != dev_entry)
	{
		return -1;
	}

	udev = interface_to_usbdev(intf);
#if BTUSB_RPM	
	message("auto suspend is enabled");  
	usb_enable_autosuspend(udev);	
	pm_runtime_set_autosuspend_delay(&(udev->dev),2000);
#endif

	dev_entry = kzalloc(sizeof(dev_data), GFP_KERNEL);
	dev_entry->intf = intf;
	dev_entry->udev = udev;
	dev_entry->pm_notifier.notifier_call = rtkbt_pm_notify;
	dev_entry->patch_entry = get_patch_entry(udev);
	list_add(&dev_entry->list_node, &dev_data_list);
	register_pm_notifier(&dev_entry->pm_notifier);

	return 0;
}

void patch_remove(struct usb_interface* intf)
{
	dev_data *dev_entry;
	struct usb_device *udev;

	udev = interface_to_usbdev(intf);
#if BTUSB_RPM
	usb_disable_autosuspend(udev);	
#endif

	dev_entry = dev_data_find(intf);
	if (NULL == dev_entry)
	{
		return;
	}
	
	message("patch_remove");
	list_del(&dev_entry->list_node);
	unregister_pm_notifier(&dev_entry->pm_notifier);
	kfree(dev_entry);
}

int download_patch(struct usb_interface* intf)
{
	dev_data		*dev_entry;
	xchange_data	*xdata = NULL;
	uint8_t			*fw_buf;
	int				ret_val;

	message("download_patch start");
	dev_entry = dev_data_find(intf);
	if (NULL == dev_entry)
	{
		ret_val = -1;
		warning("NULL == dev_entry");
		goto patch_end;
	}
	
        xdata = kzalloc(sizeof(xchange_data), GFP_KERNEL);
	if(NULL == xdata)
	{
		ret_val = -1;
		message("NULL == xdata");
		goto patch_end;	
	}
		
	init_xdata(xdata, dev_entry);
	ret_val = check_fw_version(xdata);
	if (ret_val != 0)
	{
		goto patch_end;
	}

	ret_val = get_firmware(xdata);
	if (ret_val < 0)
	{
		warning("get_firmware failed!");
		goto patch_end;
	}
	fw_buf = xdata->fw_data;

	ret_val = download_data(xdata);
	if (ret_val < 0)
	{
		warning("download_data failed!");
		goto patch_fail;
	}

	ret_val = check_fw_version(xdata);
	if (ret_val <= 0)
	{
		ret_val = -1;
		goto patch_fail;
	}

	ret_val = 0;
patch_fail:
	kfree(fw_buf);
patch_end:
	if(xdata != NULL)
	{
                if(xdata->send_pkt)
			kfree(xdata->send_pkt);
		if(xdata->rcv_pkt)	
			kfree(xdata->rcv_pkt);
        	kfree(xdata);
	}
	message("Rtk patch end %d", ret_val);
	return ret_val;
}

int set_btoff(struct usb_interface* intf)
{
	dev_data		*dev_entry;
	xchange_data	*xdata = NULL;
	int				ret_val;

	message("set_btoff");
	dev_entry = dev_data_find(intf);
	if (NULL == dev_entry)
	{
		return -1;
	}

       xdata = kzalloc(sizeof(xchange_data), GFP_KERNEL);
	if(NULL == xdata)
	{
		ret_val = -1;
		message("NULL == xdata");
              return ret_val;
       }
	
	init_xdata(xdata, dev_entry);


	xdata->cmd_hdr->opcode = cpu_to_le16(BTOFF_OPCODE);
	xdata->cmd_hdr->plen = 1;
	xdata->pkt_len = CMD_HDR_LEN + 1;
	xdata->send_pkt[CMD_HDR_LEN] = 1;
	
	ret_val = send_hci_cmd(xdata);
	if (ret_val < 0)
	{
		goto tagEnd;
	}

	ret_val = rcv_hci_evt(xdata);
	if (ret_val < 0)
	{
		goto tagEnd;
	}

tagEnd:
	if(xdata != NULL)
	{
                if(xdata->send_pkt)
			kfree(xdata->send_pkt);
		if(xdata->rcv_pkt)	
			kfree(xdata->rcv_pkt);
        	kfree(xdata);
	}

	message("set_btoff done");

	return ret_val;
}


dev_data* dev_data_find(struct usb_interface* intf)
{
	dev_data *dev_entry;

	list_for_each_entry(dev_entry, &dev_data_list, list_node)
	{
		if (dev_entry->intf == intf)
		{
			return dev_entry;
		}
	}

	return NULL;
}

patch_info* get_patch_entry(struct usb_device* udev)
{
	patch_info	*patch_entry;
	uint16_t	pid;

	patch_entry = patch_table;
	pid = le16_to_cpu(udev->descriptor.idProduct);
	message("pid = 0x%x", pid);
	while (pid != patch_entry->prod_id)
	{
		if (0 == patch_entry->prod_id) break;
		patch_entry++;
	}

	return patch_entry;
}

int rtkbt_pm_notify(
	struct notifier_block* notifier,
	ulong	pm_event,
	void*	unused)
{
	dev_data	*dev_entry;
	patch_info	*patch_entry;
	struct usb_device *udev;

	dev_entry = container_of(notifier, dev_data, pm_notifier);
	patch_entry = dev_entry->patch_entry;
	udev = dev_entry->udev;
	message("rtkbt_pm_notify pm_event =%ld",pm_event);
	switch (pm_event)
	{
		case PM_SUSPEND_PREPARE:
		case PM_HIBERNATION_PREPARE:
			patch_entry->fw_len = load_firmware(dev_entry, &patch_entry->fw_cache);
			if (patch_entry->fw_len <= 0)
			{
				message("rtkbt_pm_notify return NOTIFY_BAD");
				return NOTIFY_BAD;
			}
	
			if (!device_may_wakeup(&udev->dev))
			{
				#ifdef CONFIG_RESET_RESUME
					message("remote wakeup not support, reset_resume support ");
				#else
					dev_entry->intf->needs_binding = 1;
					message("remote wakeup not support, set intf->needs_binding = 1");
				#endif
			}
			break;

		case PM_POST_SUSPEND:
		case PM_POST_HIBERNATION:
		case PM_POST_RESTORE:
			if (patch_entry->fw_len > 0)
			{
				kfree(patch_entry->fw_cache);
				patch_entry->fw_cache = NULL;
				patch_entry->fw_len = 0;
			}
#if BTUSB_RPM
			usb_disable_autosuspend(udev);
			usb_enable_autosuspend(udev);
			pm_runtime_set_autosuspend_delay(&(udev->dev),2000);
#endif
			break;

		default:
			break;
	}

	return NOTIFY_DONE;
}

int load_firmware(dev_data* dev_entry, uint8_t** buff)
{
	const struct firmware	*fw;
	struct usb_device		*udev;
	patch_info	*patch_entry;
	char		*fw_name;
	int			fw_len = 0, ret_val;

	int config_len = 0 ,buf_len =-1;
	uint8_t* buf = *buff, *config_file_buf = NULL;
   	uint8_t* epatch_buf = NULL;

	struct rtk_epatch* epatch_info = NULL;
        uint8_t need_download_fw = 1;
	struct rtk_extension_entry patch_lmp = {0};
	struct rtk_epatch_entry current_entry = {0};
	uint16_t lmp_version ;
	uint8_t gEVersion= 0;
	
	message("load_firmware start");
	udev = dev_entry->udev;
	patch_entry = dev_entry->patch_entry;
	lmp_version = patch_entry->lmp_sub;
	

	warning("lmp_version = 0x%04x", lmp_version);
	
	fw_name = patch_entry->config_name;
	ret_val = request_firmware(&fw, fw_name, &udev->dev);
	if (ret_val < 0)
		config_len = 0;
	else
	{
		config_file_buf = kzalloc(fw->size, GFP_KERNEL);
		if (NULL == config_file_buf) goto alloc_fail;
			memcpy(config_file_buf, fw->data, fw->size);
		config_len = fw->size;
	}
	
	release_firmware(fw);
	fw_name = patch_entry->patch_name;
	ret_val = request_firmware(&fw, fw_name, &udev->dev);
	if (ret_val < 0)
	{
		fw_len = 0;
		kfree(config_file_buf);
		config_file_buf= NULL;
		goto fw_fail;
	}
	epatch_buf = kzalloc(fw->size, GFP_KERNEL);
	if (NULL == epatch_buf) goto alloc_fail;	
	memcpy(epatch_buf, fw->data, fw->size);	
	buf_len = fw->size + config_len;

	if(lmp_version == ROM_LMP_8723a)
	{
		warning("This is 8723a, use old patch style!");
		if(memcmp(epatch_buf, RTK_EPATCH_SIGNATURE, 8) == 0)
		{
			warning("8723as Check signature error!");
			need_download_fw = 0;
	}
		else
		{
			if (!(buf = kzalloc(buf_len, GFP_KERNEL))) {
				warning("Can't alloc memory for fw&config");
				buf_len = -1;
			}
			else
			{
				message("8723as, fw copy direct");
				memcpy(buf,epatch_buf,buf_len);
				kfree(epatch_buf);
				epatch_buf = NULL;
				if (config_len)
				{
					memcpy(&buf[buf_len - config_len], config_file_buf, config_len);
				}	
			}
		}
	}

	else
	{
		warning("This is not 8723a, use new patch style!");
		//Get version from ROM
		gEVersion = rtk_get_eversion(dev_entry);  //gEVersion is set.
		message("gEVersion=%d", gEVersion);

		//check Extension Section Field 
		if(memcmp(epatch_buf + buf_len-config_len-4 ,Extension_Section_SIGNATURE,4) != 0)
		{
			warning("Check Extension_Section_SIGNATURE error! do not download fw");
			need_download_fw = 0;
		}
		else
		{
			uint8_t *temp;  
			temp = epatch_buf+buf_len-config_len-5;
			do{
				if(*temp == 0x00)
				{
					patch_lmp.opcode = *temp;
					patch_lmp.length = *(temp-1);
					if ((patch_lmp.data = kzalloc(patch_lmp.length, GFP_KERNEL)))
					{
						memcpy(patch_lmp.data,temp-2,patch_lmp.length);
					}
					message("opcode = 0x%x",patch_lmp.opcode); 
					message("length = 0x%x",patch_lmp.length);
					message("data = 0x%x",*(patch_lmp.data));
					break;
				}
				temp -= *(temp-1)+2;
			}while(*temp != 0xFF);

			if(lmp_version != project_id[*(patch_lmp.data)])
			{
				warning("lmp_version is %x, project_id is %x, does not match!!!",lmp_version,project_id[*(patch_lmp.data)]);
				need_download_fw = 0;
			}
			else
			{
				message("lmp_version is %x, project_id is %x, match!",lmp_version, project_id[*(patch_lmp.data)]);
		
				if(memcmp(epatch_buf, RTK_EPATCH_SIGNATURE, 8) != 0)
				{
					message("Check signature error!");
					need_download_fw = 0;
				}
				else
				{
					int i = 0;
					epatch_info = (struct rtk_epatch*)epatch_buf;
					message("fm_version = 0x%x",epatch_info->fm_version); 
					message("number_of_total_patch = %d",epatch_info->number_of_total_patch);
			
					//get right epatch entry
					for(i=0; i<epatch_info->number_of_total_patch; i++)
					{
						if(*(uint16_t*)(epatch_buf+14+2*i) == gEVersion + 1)
						{
							current_entry.chipID = gEVersion + 1;
							current_entry.patch_length = *(uint16_t*)(epatch_buf+14+2*epatch_info->number_of_total_patch+2*i);
							current_entry.start_offset = *(uint32_t*)(epatch_buf+14+4*epatch_info->number_of_total_patch+4*i);
							break;
						}
					}
					message("chipID = %d",current_entry.chipID); 
					message("patch_length = 0x%x",current_entry.patch_length); 
					message("start_offset = 0x%x",current_entry.start_offset); 
									
					//get right eversion patch: buf, buf_len
					buf_len = current_entry.patch_length + config_len;
					message("buf_len = 0x%x",buf_len);
									
					if (!(buf = kzalloc(buf_len, GFP_KERNEL))) {
						warning("Can't alloc memory for multi fw&config");
						buf_len = -1;
					}
					else
					{
						memcpy(buf,&epatch_buf[current_entry.start_offset],current_entry.patch_length);
						memcpy(&buf[current_entry.patch_length-4],&epatch_info->fm_version,4);
					}
					kfree(epatch_buf);
						epatch_buf = NULL;

					if (config_len)
					{
						memcpy(&buf[buf_len - config_len], config_file_buf, config_len);
					}	
				}															
			}
		}					
	}					
                
       if (config_file_buf)
        	 kfree(config_file_buf);
      
	warning("Fw:%s exists, config file:%s exists", (buf_len > 0) ? "":"not", (config_len>0)?"":"not");
	if (buf && (buf_len > 0) && (need_download_fw))
	{
		fw_len = buf_len;	
		*buff = buf;
	}

	message("load_firmware done");

alloc_fail:
	release_firmware(fw);
fw_fail:
	return fw_len;
}

void init_xdata(
	xchange_data*	xdata,
	dev_data*		dev_entry)
{
	memset(xdata, 0, sizeof(xchange_data));
	xdata->dev_entry = dev_entry;
	xdata->pipe_in = usb_rcvintpipe(dev_entry->udev, INTR_EP);
	xdata->pipe_out = usb_sndctrlpipe(dev_entry->udev, CTRL_EP);
        xdata->send_pkt = kzalloc(PKT_LEN, GFP_KERNEL);
	xdata->rcv_pkt = kzalloc(PKT_LEN, GFP_KERNEL);	   
	xdata->cmd_hdr = (struct hci_command_hdr*)(xdata->send_pkt);
	xdata->evt_hdr = (struct hci_event_hdr*)(xdata->rcv_pkt);
	xdata->cmd_cmp = (struct hci_ev_cmd_complete*)(xdata->rcv_pkt + EVT_HDR_LEN);
	xdata->req_para = xdata->send_pkt + CMD_HDR_LEN;
	xdata->rsp_para = xdata->rcv_pkt + EVT_HDR_LEN + CMD_CMP_LEN;
}

int check_fw_version(xchange_data* xdata)
{
	struct hci_rp_read_local_version *read_ver_rsp;
	patch_info	*patch_entry;
	int			ret_val;

	xdata->cmd_hdr->opcode = cpu_to_le16(HCI_OP_READ_LOCAL_VERSION);
	xdata->cmd_hdr->plen = 0;
	xdata->pkt_len = CMD_HDR_LEN;

	ret_val = send_hci_cmd(xdata);
	if (ret_val < 0)
	{
		goto version_end;
	}

	ret_val = rcv_hci_evt(xdata);
	if (ret_val < 0)
	{
		goto version_end;
	}

	patch_entry = xdata->dev_entry->patch_entry;
	read_ver_rsp = (struct hci_rp_read_local_version*)(xdata->rsp_para);
	message("check_fw_version : read_ver_rsp->lmp_subver = 0x%x",read_ver_rsp->lmp_subver);
	message("check_fw_version : patch_entry->lmp_sub = 0x%x",patch_entry->lmp_sub);
	if (patch_entry->lmp_sub != read_ver_rsp->lmp_subver)
	{
		return 1;
	}

	ret_val = 0;
version_end:
	return ret_val;
}

uint8_t rtk_get_eversion(dev_data * dev_entry)
{
	struct rtk_eversion_evt *eversion;
	patch_info	*patch_entry;
	int			ret_val = 0;
	xchange_data* xdata = NULL;
	
	if(global_eversion != 0xff)
		return global_eversion;
	
	xdata = kzalloc(sizeof(xchange_data), GFP_KERNEL);
	if(NULL == xdata)
	{
		ret_val = 0;
		message("NULL == xdata");
              return ret_val;
       }
	
	init_xdata(xdata, dev_entry);

	xdata->cmd_hdr->opcode = cpu_to_le16(HCI_VENDOR_READ_RTK_ROM_VERISION);
	xdata->cmd_hdr->plen = 0;
	xdata->pkt_len = CMD_HDR_LEN;

	ret_val = send_hci_cmd(xdata);
	if (ret_val < 0)
	{
		goto version_end;
	}

	ret_val = rcv_hci_evt(xdata);
	if (ret_val < 0)
	{
		goto version_end;
	}

	patch_entry = xdata->dev_entry->patch_entry;
	eversion = (struct rtk_eversion_evt*)(xdata->rsp_para);
	message("rtk_get_eversion : eversion->status = 0x%x, eversion->version = 0x%x",eversion->status, eversion->version);
	if (eversion->status)
	{
		ret_val = 0;
		global_eversion = 0;
	}
	else 
	{
		ret_val =  eversion->version;
		global_eversion = eversion->version;
	}

version_end:
	if(xdata != NULL)
	{
                if(xdata->send_pkt)
			kfree(xdata->send_pkt);
		if(xdata->rcv_pkt)	
			kfree(xdata->rcv_pkt);
        	kfree(xdata);
	}
	return ret_val;

}

int get_firmware(xchange_data* xdata)
{
	dev_data	*dev_entry;
	patch_info	*patch_entry;
	message("get_firmware start");

	dev_entry = xdata->dev_entry;
	patch_entry = dev_entry->patch_entry;
	if (patch_entry->fw_len > 0)
	{
		xdata->fw_data = kzalloc(patch_entry->fw_len, GFP_KERNEL);
		if (NULL == xdata->fw_data) return -ENOMEM;
		memcpy(xdata->fw_data, patch_entry->fw_cache, patch_entry->fw_len);
		xdata->fw_len = patch_entry->fw_len;
	}
	else
	{
		xdata->fw_len = load_firmware(dev_entry, &xdata->fw_data);
		if (xdata->fw_len <= 0) return -1;
	}
	message("get_firmware done");
	return 0;
}

int download_data(xchange_data* xdata)
{
	download_cp *cmd_para;
	download_rp *evt_para;
	uint8_t		*pcur;
	int			pkt_len, frag_num, frag_len;
	int			i, ret_val;

	message("download_data start");
	
	cmd_para = (download_cp*)xdata->req_para;
	evt_para = (download_rp*)xdata->rsp_para;
	pcur = xdata->fw_data;
	pkt_len = CMD_HDR_LEN + sizeof(download_cp);
	frag_num = xdata->fw_len / PATCH_SEG_MAX + 1;
	frag_len = PATCH_SEG_MAX;
	
	for (i = 0; i < frag_num; i++)
	{
		cmd_para->index = i;
		if (i == (frag_num - 1))
		{
			cmd_para->index |= DATA_END;
			frag_len = xdata->fw_len % PATCH_SEG_MAX;
			pkt_len -= (PATCH_SEG_MAX - frag_len);
		}
		xdata->cmd_hdr->opcode = cpu_to_le16(DOWNLOAD_OPCODE);
		xdata->cmd_hdr->plen = sizeof(uint8_t) + frag_len;
		xdata->pkt_len = pkt_len;
		memcpy(cmd_para->data, pcur, frag_len);

		ret_val = send_hci_cmd(xdata);
		if (ret_val < 0)
		{
			return ret_val;
		}

		ret_val = rcv_hci_evt(xdata);
		if (ret_val < 0)
		{
			return ret_val;
		}
		if (0 != evt_para->status)
		{
			return -1;
		}

		pcur += PATCH_SEG_MAX;
	}

	message("download_data done");
	return xdata->fw_len;
}

int send_hci_cmd(xchange_data* xdata)
{
	int ret_val;

	ret_val = usb_control_msg(
		xdata->dev_entry->udev, xdata->pipe_out,
		0, USB_TYPE_CLASS, 0, 0,
		(void*)(xdata->send_pkt),
		xdata->pkt_len, MSG_TO);

	return ret_val;
}

int rcv_hci_evt(xchange_data* xdata)
{
	int ret_len = 0, ret_val = 0;
	int i;   // Added by Realtek

	while (1)
	{
		
		// **************************** Modifed by Realtek (begin)
		for(i = 0; i < 5; i++)   // Try to send USB interrupt message 5 times.
		{
		ret_val = usb_interrupt_msg(
			xdata->dev_entry->udev, xdata->pipe_in,
			(void*)(xdata->rcv_pkt), PKT_LEN,
			&ret_len, MSG_TO);
			if(ret_val >= 0)
				break;
		}
		// **************************** Modifed by Realtek (end)

		if (ret_val < 0)
		{
			return ret_val;
		}

		
	      if (CMD_CMP_EVT == xdata->evt_hdr->evt)
	       {   
	           if (xdata->cmd_hdr->opcode == xdata->cmd_cmp->opcode)
	              return ret_len;
	       }


	}
}
