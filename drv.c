#include    <linux/kernel.h>
#include    <linux/errno.h>
#include    <linux/init.h>
#include    <linux/slab.h>
#include    <linux/module.h>
#include    <linux/kref.h>
#include    <linux/mutex.h>
#include    <linux/uaccess.h>
#include    <linux/usb.h>
#include    "patch.h"
#include    "wb.h"
#include    "queue.h"
#include    "zdebug.h"

#define     USE_PATCH

#define     VERSION     "0.1"

#define BCSP_CHANNEL_HCI            (5)
#define BCSP_CHANNEL_ACL            (6)
#define BCSP_CHANNEL_SCO            (7)

#define MAX_PACKET_SIZE             1024
#define MAX_ISOC_FRAMES             10
#define WRITES_IN_FLIGHT            7

#define WB_INTR_RUNNING             0
#define WB_BULK_RUNNING             1
#define WB_ISOC_RUNNING             2
#define WB_HCI_RUNNING              4

#define USB_MINOR_BASE              192
#define URB_CANCELING_DELAY_MS      10

#define SHARD_NR                    3
#define SHARD_INDEX(x)              ((x) - BCSP_CHANNEL_HCI)

#pragma pack(1)    
struct host_packet{
    uint8_t  channel;
    int8_t  buffer[0];
};

struct hci_command_packet{
    uint16_t opcode;
    uint8_t length;
    int8_t  buffer[0];
};

struct sco_packet{
    uint16_t handle;
    uint8_t length;
    int8_t  buffer[0];
};

struct hci_event_packet{
    uint8_t  event;
    uint8_t length;
    int8_t  buffer[0];
};

struct acl_packet{
    uint16_t  handle;
    uint16_t length;
    int8_t   buffer[0];
};

#pragma pack()

static const size_t packet_hdr_lengths[SHARD_NR] = {
    [SHARD_INDEX(BCSP_CHANNEL_HCI)] = sizeof(struct hci_event_packet),
    [SHARD_INDEX(BCSP_CHANNEL_ACL)] = sizeof(struct acl_packet),
    [SHARD_INDEX(BCSP_CHANNEL_SCO)] = sizeof(struct sco_packet),
};

#define PACKET_HDR_LENGTH(x)    packet_hdr_lengths[SHARD_INDEX(x)]

/*! USB设备匹配 !*/
static const struct usb_device_id wb_table [] ={
    {
        .match_flags        = USB_DEVICE_ID_MATCH_VENDOR | USB_DEVICE_ID_MATCH_INT_INFO,
        .idVendor           = 0x0bda,
        .bInterfaceClass    = 0xe0,
        .bInterfaceSubClass = 0x01,
        .bInterfaceProtocol = 0x01,
    },
    {},
};
MODULE_DEVICE_TABLE(usb,wb_table);

struct usb_wb{
    struct  usb_device              *udev;      /*! !*/
    struct  usb_interface           *intf;
    struct  usb_interface           *isoc;
    struct  usb_anchor              submit;         /*! 加入anchor相当于加入一个组织,统一听从组织安排 !*/
    struct  usb_anchor              isoc_anchor;   /*! 加入anchor相当于加入一个组织,统一听从组织安排 !*/
	struct  work_struct             work;
	struct  work_struct             sleep;
	struct  semaphore	            limit_sem;
    struct  mutex                   io_mutex;
    spinlock_t                      complete_spin;  /*! 数据接收锁 !*/
    struct  completion              read_completion;
    struct  usb_endpoint_descriptor *intr_ep;
    struct  usb_endpoint_descriptor *isoc_tx_ep;
    struct  usb_endpoint_descriptor *isoc_rx_ep;
    struct  usb_endpoint_descriptor *bulk_tx_ep;
    struct  usb_endpoint_descriptor *bulk_rx_ep;
    void                            *queue;
    int                             isoc_altsetting;
    unsigned long                   flags;
    struct{
        unsigned int                total;
        unsigned int                remain;
        Element                     *element;
    }shard[SHARD_NR];
} wb_driver;

static int      wb_probe(struct usb_interface *intf,const struct usb_device_id *id);
static void     wb_disconnect(struct usb_interface *intf);
static int      wb_open(struct inode *inode,struct file *file);
static int      wb_release(struct inode *inode,struct file *file);
static ssize_t  wb_read (struct file *file,char *buffer,size_t count,loff_t *opps);
static ssize_t  wb_write(struct file *file,const char *buffer,size_t count,loff_t *opps);
static long     wb_ioctl(struct file *file,unsigned int cmd,unsigned long arg);
static int      wb_flush(struct file *file,fl_owner_t id);
static int      wb_submit_isoc_urb(struct usb_wb *dev,int mem_flags);

static struct usb_driver usb_device = {
    .name                   = "gwb",
    .probe                  = wb_probe,
    .disconnect             = wb_disconnect,
    .id_table               = wb_table,
    .supports_autosuspend   = 1,
#if 0
#if LINUX_VERSION_CODE > KERNEL_VERSION(3,5,0)
    .disable_hub_initiated_lpm = 1,
#endif
#endif
};

static struct file_operations wb_fops = {
    .owner          = THIS_MODULE,
    .read           = wb_read,
    .write          = wb_write,
    .open           = wb_open,
    .release        = wb_release,
    .flush          = wb_flush,
    .unlocked_ioctl = wb_ioctl,
    .llseek         = noop_llseek,
};

static struct usb_class_driver wb_class = {
    .name       = "gwb%d",
    .fops       = &wb_fops,
    .minor_base = USB_MINOR_BASE,
};

static /*inline*/ int packing(struct usb_wb *dev,Element *element,const int channel){
    int rv = 0;
    struct host_packet *packet;
    if(channel == BCSP_CHANNEL_ACL) message();
    if(!element) return -ENOMEM;
    packet = (void *)element->packet;
    packet->channel = channel;
    rv = enQueue(dev->queue,element);
    if(rv)
        kfree(element);
    else
        complete(&dev->read_completion);
    return rv;
}

static /*inline*/ size_t packet_length(void *packet,const int channel){
    if(!packet) return 0;
    switch(channel){
        case BCSP_CHANNEL_HCI: 
            return sizeof(struct hci_event_packet) + ((struct hci_event_packet*)packet)->length;
        case BCSP_CHANNEL_ACL: 
            return sizeof(struct acl_packet) + le16_to_cpu(((struct acl_packet*)packet)->length);
        case BCSP_CHANNEL_SCO: 
            return sizeof(struct sco_packet) + ((struct sco_packet*)packet)->length;
    }
    return 0;
}

static /*inline*/ int reassembly(struct usb_wb *dev,void *data,size_t length, const int status,const int channel){
    uint8_t     len;
    uint32_t    offset;
    struct  host_packet *packet;
    const int   index = SHARD_INDEX(channel);
    if(status){
        if(dev->shard[index].element != NULL){
            kfree(dev->shard[index].element);
            dev->shard[index].element = NULL;
            dev->shard[index].total  = 0;
        }
        if(length <= dev->shard[index].remain){
            dev->shard[index].remain -= length;
            return status;
        }else{
            length -= dev->shard[index].remain;
            data += length;
        }
    }
    if((dev->shard[index].element == NULL) && (dev->shard[index].remain)){
        if(length <= dev->shard[index].remain){
            dev->shard[index].remain -= length;
            return -1;
        }else{
            length -= dev->shard[index].remain;
            data += length;
        }
    }
    while(length){
        len = 0;
        if(dev->shard[index].element == NULL){
            if(length >= PACKET_HDR_LENGTH(channel)){
                dev->shard[index].total   = packet_length(data,channel);
                dev->shard[index].remain  = dev->shard[index].total;
            }else{
                return -EILSEQ;
            }
            dev->shard[index].element = kzalloc(dev->shard[index].total + sizeof(struct host_packet) + sizeof(Element),GFP_ATOMIC);
            if(!dev->shard[index].element) return -ENOMEM;
            dev->shard[index].element->length = sizeof(struct host_packet) + dev->shard[index].total;
        }
        packet = (void*)dev->shard[index].element->packet;
        len = min(dev->shard[index].remain,length);
        offset = dev->shard[index].total - dev->shard[index].remain;
        memcpy(((void*)(packet->buffer)) + offset,data,len);
        dev->shard[index].remain -= len;
        if(!dev->shard[index].remain){
            packing(dev,dev->shard[index].element,channel);
            dev->shard[index].element    = NULL;
        }
        length -= len;
        data += len;
    }
    return 0;
}

static void wb_isoc_complete(struct urb *urb){
    struct usb_wb *dev = urb->context;
    int rv;

    if(!test_bit(WB_HCI_RUNNING,&dev->flags)) return;
    spin_lock(&dev->complete_spin);
    if(!urb->status){
        for(int i = 0;i < urb->number_of_packets;i++){
            unsigned long offset = urb->iso_frame_desc[i].offset;
            unsigned long length = urb->iso_frame_desc[i].actual_length;
            reassembly(dev,urb->transfer_buffer + offset,length,urb->iso_frame_desc[i].status,BCSP_CHANNEL_SCO);
        }
    }
    spin_unlock(&dev->complete_spin);
    if(!test_bit(WB_ISOC_RUNNING,&dev->flags)) return;
    usb_anchor_urb(urb,&dev->isoc_anchor);
    rv = usb_submit_urb(urb,GFP_ATOMIC);
    if(rv)
        usb_unanchor_urb(urb);
}

static void wb_intr_complete(struct urb *urb){
    struct usb_wb *dev = urb->context;
    int rv;
    if(!test_bit(WB_HCI_RUNNING,&dev->flags)) return ;
    spin_lock(&dev->complete_spin);
    if(!urb->status){
        reassembly(dev,urb->transfer_buffer,urb->actual_length,urb->status,BCSP_CHANNEL_HCI);
    }
    spin_unlock(&dev->complete_spin);
    if(!test_bit(WB_INTR_RUNNING,&dev->flags)) return;
    usb_anchor_urb(urb,&dev->submit);
    usb_mark_last_busy(dev->udev);
    rv = usb_submit_urb(urb,GFP_ATOMIC);
    if(rv)
        usb_unanchor_urb(urb);
}

static void wb_bulk_complete(struct urb *urb){
    struct usb_wb *dev = urb->context;
    int rv;
    if(!test_bit(WB_HCI_RUNNING,&dev->flags)) return ;
    spin_lock(&dev->complete_spin);
    if(!urb->status){
        message("length : %d,%d",urb->actual_length,((struct acl_packet *)urb->transfer_buffer)->length);
        reassembly(dev,urb->transfer_buffer,urb->actual_length,urb->status,BCSP_CHANNEL_ACL);
    }
    spin_unlock(&dev->complete_spin);
    if(!test_bit(WB_BULK_RUNNING,&dev->flags)) return;
    usb_anchor_urb(urb,&dev->submit);
    usb_mark_last_busy(dev->udev);
    rv = usb_submit_urb(urb,GFP_ATOMIC);
    if(rv)
        usb_unanchor_urb(urb);
    return;
}


static /*inline*/ int __set_isoc_interface(struct usb_wb *dev, int altsetting) {
	struct usb_interface *intf = dev->isoc;
	struct usb_endpoint_descriptor *ep_desc;
	int i, err;
	if (!dev->isoc) return -ENODEV;
	err = usb_set_interface(dev->udev, 1, altsetting);
	if (err < 0) {
		return err;
	}
	dev->isoc_altsetting = altsetting;
	dev->isoc_tx_ep = NULL;
	dev->isoc_rx_ep = NULL;

	for (i = 0; i < intf->cur_altsetting->desc.bNumEndpoints; i++) {
		ep_desc = &intf->cur_altsetting->endpoint[i].desc;

		if (!dev->isoc_tx_ep && usb_endpoint_is_isoc_out(ep_desc)) {
			dev->isoc_tx_ep = ep_desc;
			continue;
		}

		if (!dev->isoc_rx_ep && usb_endpoint_is_isoc_in(ep_desc)) {
			dev->isoc_rx_ep = ep_desc;
			continue;
		}
	}
	if (!dev->isoc_tx_ep || !dev->isoc_rx_ep) {
		return -ENODEV;
	}
	return 0;
}

static int wb_flush(struct file *file,fl_owner_t id){
    struct usb_wb   *dev = file->private_data;
    mdelay(URB_CANCELING_DELAY_MS);
    usb_kill_anchored_urbs(&dev->submit);
    return 0;
}

static long wb_ioctl(struct file *file,unsigned int cmd,unsigned long arg){
    struct usb_wb *dev = file->private_data;
    switch(cmd){
        case SCO_WORK:
            schedule_work(&dev->work);
            break;
        case SCO_SLEEP:
            schedule_work(&dev->sleep);
            break;
        default :
            return -1;
    }
    return 0;
}

static void wb_work(struct work_struct *work){
    struct usb_wb   *dev = container_of(work,struct usb_wb,work);
    if(!dev->isoc_altsetting != 2){
        clear_bit(WB_ISOC_RUNNING,&dev->flags);
        mdelay(URB_CANCELING_DELAY_MS);
        usb_kill_anchored_urbs(&dev->isoc_anchor);
        if(__set_isoc_interface(dev,2) < 0){
            return;
        }
    }
    if(!test_and_set_bit(WB_ISOC_RUNNING,&dev->flags)){
        if(wb_submit_isoc_urb(dev,GFP_KERNEL) < 0){
            return;
        }
        wb_submit_isoc_urb(dev,GFP_KERNEL);
    }
}

static void wb_sleep(struct work_struct *work){
    struct usb_wb   *dev = container_of(work,struct usb_wb,sleep);
    clear_bit(WB_ISOC_RUNNING,&dev->flags);
    mdelay(URB_CANCELING_DELAY_MS);
    usb_kill_anchored_urbs(&dev->isoc_anchor);
    __set_isoc_interface(dev,0);
}

static /*inline*/ void __fill_isoc_descriptor(struct urb *urb, int len, int mtu) {
	int i, offset = 0;

	for (i = 0; i < MAX_ISOC_FRAMES && len >= mtu;
					i++, offset += mtu, len -= mtu) {
		urb->iso_frame_desc[i].offset = offset;
		urb->iso_frame_desc[i].length = mtu;
	}

	if (len && i < MAX_ISOC_FRAMES) {
		urb->iso_frame_desc[i].offset = offset;
		urb->iso_frame_desc[i].length = len;
		i++;
	}
	urb->number_of_packets = i;
}

static int wb_submit_isoc_urb(struct usb_wb *dev,int mem_flags){
    struct urb *urb;
    unsigned char *buf;
    unsigned int pipe;
    int err, size;
    message();

    if (!dev->isoc_rx_ep) return -ENODEV;

    urb = usb_alloc_urb(MAX_ISOC_FRAMES, mem_flags);
    if (!urb) return -ENOMEM;

    size = le16_to_cpu(dev->isoc_rx_ep->wMaxPacketSize) *
        MAX_ISOC_FRAMES;

    buf = kmalloc(size, mem_flags);
    if (!buf) {
        err = -ENOMEM;
        goto _free_urb;
    }

    pipe = usb_rcvisocpipe(dev->udev, dev->isoc_rx_ep->bEndpointAddress);

    urb->dev      = dev->udev;
    urb->pipe     = pipe;
    urb->context  = dev;
    urb->complete = wb_isoc_complete;
    urb->interval = dev->isoc_rx_ep->bInterval;

    urb->transfer_flags  = URB_FREE_BUFFER | URB_ISO_ASAP;
    urb->transfer_buffer = buf;
    urb->transfer_buffer_length = size;

    __fill_isoc_descriptor(urb, size,
            le16_to_cpu(dev->isoc_rx_ep->wMaxPacketSize));

    usb_anchor_urb(urb, &dev->isoc_anchor);

    err = usb_submit_urb(urb, mem_flags);
    if (err < 0) {
        usb_unanchor_urb(urb);
    }
_free_urb:
    usb_free_urb(urb);
    return err;
}

static int wb_submit_intr_urb(struct usb_wb *dev,int mem_flags){
    struct urb  *urb;
    void        *buff;
    unsigned int pipe;
    int size;
    int rv;
    if(!dev->intr_ep) return -ENODEV;
    urb = usb_alloc_urb(0,mem_flags);
    if(!urb) return -ENOMEM;
    size = le16_to_cpu(dev->intr_ep->wMaxPacketSize);
    buff = kmalloc(size,mem_flags);
    if(!buff) goto _free_urb;
    pipe = usb_rcvintpipe(dev->udev,dev->intr_ep->bEndpointAddress);
    usb_fill_int_urb(urb,dev->udev,pipe,buff,size,wb_intr_complete,dev,dev->intr_ep->bInterval);
    urb->transfer_flags |= URB_FREE_BUFFER;
    ///*! ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ !*/
    //usb_mark_last_busy(dev->udev);
    usb_anchor_urb(urb,&dev->submit);
    rv = usb_submit_urb(urb,mem_flags);
    if(rv){
        usb_unanchor_urb(urb);
        goto _free_urb;     /*! buff,由usb_free_urb释放 !*/
    }
_free_urb:
    usb_free_urb(urb);
    return rv;
}

static int wb_submit_bulk_urb(struct usb_wb *dev,int mem_flags){
    struct urb  *urb;
    void        *buff;
    unsigned int pipe;
    const int size = MAX_PACKET_SIZE;
    int rv;
    if(!dev->bulk_rx_ep) return -ENODEV;
    urb = usb_alloc_urb(0,mem_flags);
    if(!urb) return -ENOMEM;
    buff = kmalloc(size,mem_flags);
    if(!buff) goto _free_urb;
    pipe = usb_rcvbulkpipe(dev->udev,dev->bulk_rx_ep->bEndpointAddress);
    usb_fill_bulk_urb(urb,dev->udev,pipe,buff,size,wb_bulk_complete,dev);
    urb->transfer_flags |= URB_FREE_BUFFER;
    /*! ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ !*/
    usb_mark_last_busy(dev->udev);
    usb_anchor_urb(urb,&dev->submit);
    rv = usb_submit_urb(urb,mem_flags);
    if(rv){
        usb_unanchor_urb(urb);
        goto _free_urb;     /*! buff,由usb_free_urb释放 !*/
    }
_free_urb:
    usb_free_urb(urb);
    return rv;
}

static void wb_write_complete(struct urb *urb){
    struct usb_wb *dev = urb->context;
    if(!test_bit(WB_HCI_RUNNING,&dev->flags))return;
    kfree(urb->setup_packet);
	up(&dev->limit_sem);
}
static void wb_isoc_write_complete(struct urb *urb){
    struct usb_wb *dev = urb->context;
    message();
    if(!test_bit(WB_HCI_RUNNING,&dev->flags)) return;
    kfree(urb->setup_packet);
	up(&dev->limit_sem);
}

static ssize_t wb_read (struct file *file,char *buffer,size_t count,loff_t *ppos){
    struct usb_wb   *dev = file->private_data;
    struct element  *element = NULL;
    int rv;
    if(!(dev->bulk_rx_ep || dev->intr_ep || dev->isoc_rx_ep) 
            || !count || !test_bit(WB_HCI_RUNNING,&dev->flags)) return 0;
    rv = mutex_lock_interruptible(&dev->io_mutex);
    if(rv) return rv;
    if(!dev->intf) {
        rv = -ENODEV;
        goto exit;
    }
    /*! FIXME : 这里的解锁会出现问题,但是不解锁会被锁死,暂且先解锁,解决掉零一个问题,在查这个问题 !*/
    mutex_unlock(&dev->io_mutex);
    rv = wait_for_completion_interruptible(&dev->read_completion);
    if(rv){
        goto exit;
    }
    element = deQueue(dev->queue);
    if(!element) goto exit;
    rv = min(element->length,count);
    rv = rv - copy_to_user(buffer,element->packet,rv);
    kfree(element);
exit:
    return rv;
}

static ssize_t wb_write(struct file *file,const char *buffer,size_t count,loff_t *ppos){
    struct usb_wb *dev = file->private_data;
    struct urb *urb = NULL;
    const struct host_packet *src = (void*)buffer;
    void *dest;
    unsigned pipe;
    int rv;
    const size_t writesize = count - 1;
    if(!test_bit(WB_HCI_RUNNING,&dev->flags)) return -EBUSY;
    if(!(dev->bulk_rx_ep || dev->intr_ep || dev->isoc_rx_ep) || !count || !test_bit(WB_HCI_RUNNING,&dev->flags)) return 0;
    if(!(file->f_flags & O_NONBLOCK)){
        if(down_interruptible(&dev->limit_sem)){
            rv = -ERESTARTSYS;
            goto exit;
        }
    }else{
        if(down_trylock(&dev->limit_sem)){
            rv = -EAGAIN;
            goto exit;
        }
    }
    dest = kmalloc(writesize,GFP_KERNEL);
    if(!dest){
        rv = -ENOMEM;
        goto error;
    }
    if(copy_from_user(dest,src->buffer,writesize)){
        rv = -EFAULT;
        goto error;
    }
    mutex_lock(&dev->io_mutex);
    switch(src->channel){
        case BCSP_CHANNEL_HCI:
            {
                struct usb_ctrlrequest *dr;
                urb = usb_alloc_urb(0,GFP_KERNEL);
                if(!urb){
                    rv = -ENOMEM;
                    goto error_unlock;
                }
                dr = kmalloc(sizeof(*dr),GFP_ATOMIC);
                if(!dr){
                    rv = -ENOMEM;
                    goto error_unlock;
                }
                *dr = (struct usb_ctrlrequest){
                    .bRequestType   = USB_TYPE_CLASS,
                        .bRequest       = 0,
                        .wIndex         = 0,
                        .wValue         = 0,
                        .wLength        = __cpu_to_le16(writesize),
                };
                pipe = usb_sndctrlpipe(dev->udev,0x00);
                usb_fill_control_urb(urb,dev->udev,pipe,(void*)dr,dest,writesize,wb_write_complete,dev);
            }
            break;
        case BCSP_CHANNEL_ACL:
            {
                urb = usb_alloc_urb(0,GFP_KERNEL);
                if(!urb){
                    rv = -ENOMEM;
                    goto error_unlock;
                }
                pipe = usb_sndbulkpipe(dev->udev,dev->bulk_tx_ep->bEndpointAddress);
                usb_fill_bulk_urb(urb,dev->udev,pipe,dest,writesize,wb_write_complete,dev);
            }
            break;
        case BCSP_CHANNEL_SCO:
            {
                if(!test_bit(WB_ISOC_RUNNING,&dev->flags)){
                    schedule_work(&dev->work);
                    rv = -ENODEV;
                    goto error_unlock;
                }
                urb = usb_alloc_urb(MAX_ISOC_FRAMES,GFP_KERNEL);
                if(!urb){
                    rv = -ENOMEM;
                    goto error_unlock;
                }
                pipe = usb_sndisocpipe(dev->udev,dev->isoc_tx_ep->bEndpointAddress);
                usb_fill_int_urb(urb,dev->udev,pipe,dest,writesize,wb_isoc_write_complete,dev,dev->isoc_tx_ep->bInterval);
                urb->transfer_flags = URB_ISO_ASAP;
                __fill_isoc_descriptor(urb,writesize,le16_to_cpu(dev->isoc_tx_ep->wMaxPacketSize));
            }
            break;
        default:
            rv = -EILSEQ;
            goto error_unlock;
    }
    usb_anchor_urb(urb,&dev->submit);
    rv = usb_submit_urb(urb,GFP_ATOMIC);
    if(rv){
        kfree(urb->setup_packet);
        usb_unanchor_urb(urb);
    }else{
        usb_mark_last_busy(dev->udev);
    }
    usb_free_urb(urb);
    mutex_unlock(&dev->io_mutex);
    return writesize;
error_unlock:
    mutex_unlock(&dev->io_mutex);
    kfree(dest);
error:
    if(urb)
        usb_free_urb(urb);
    up(&dev->limit_sem);
exit:
    return rv;
}

static int wb_open(struct inode *inode,struct file *file){
    struct usb_wb   *dev;
    struct usb_interface *intf;
    int             subminor = iminor(inode);
    int rv;

    intf = usb_find_interface(&usb_device,subminor);
    if(!intf){
        warning("Can't find device for minor %d",subminor);
        return -ENODEV;
    }
    dev = usb_get_intfdata(intf);
    if(!dev){
        warning("Not get data for interface");
        return -ENODEV;
    }
    rv = usb_autopm_get_interface(intf);
    if(rv){
        warning("PM get for interface");
        return rv;
    }

#ifdef  USE_PATCH
    rv = download_patch(intf);
    if(rv < 0) return rv;
#endif

    dev->intf->needs_remote_wakeup = 1;
    file->private_data = dev;
    if(test_and_set_bit(WB_HCI_RUNNING,&dev->flags)) goto done;
    if(test_and_set_bit(WB_INTR_RUNNING,&dev->flags)) goto done;
    rv = wb_submit_intr_urb(dev,GFP_KERNEL);
    if(0 > rv){
        warning("submit intr failed");
        goto failed;
    }
    rv = wb_submit_bulk_urb(dev,GFP_KERNEL);
    if(0 > rv){
        warning("submit bulk failed");
        goto failed;
    }
    if(test_and_set_bit(WB_BULK_RUNNING,&dev->flags)) goto done;
done:
    usb_autopm_put_interface(dev->intf);
    return 0;
failed:
    mdelay(URB_CANCELING_DELAY_MS);
    usb_kill_anchored_urbs(&dev->submit);
    clear_bit(WB_INTR_RUNNING,&dev->flags);
    clear_bit(WB_HCI_RUNNING,&dev->flags);
    usb_autopm_put_interface(dev->intf);
    return rv;
}

static int wb_release(struct inode *inode,struct file *file){
    struct usb_wb   *dev;
    int             rv;
    dev = file->private_data;
    if(!dev){
        warning("");
        return -ENODEV;
    }
    if(!test_and_clear_bit(WB_HCI_RUNNING,&dev->flags)) return 0;
	cancel_work_sync(&dev->work);
	cancel_work_sync(&dev->sleep);
    clear_bit(WB_ISOC_RUNNING,&dev->flags);
    clear_bit(WB_BULK_RUNNING,&dev->flags);
    clear_bit(WB_INTR_RUNNING,&dev->flags);
    usb_kill_anchored_urbs(&dev->submit);

    mutex_lock(&dev->io_mutex);
    if(dev->intf){
        rv = usb_autopm_get_interface(dev->intf);
        if(rv){
            warning("");
            goto exit;
        }
        dev->intf->needs_remote_wakeup = 0;
        usb_autopm_put_interface(dev->intf);
    }
exit:
    mutex_unlock(&dev->io_mutex);
    return 0;
}

static int wb_probe(struct usb_interface *intf,const struct usb_device_id *id){
    struct usb_wb                   *dev;
    struct usb_device               *udev;
	struct usb_host_interface       *iface_desc;
    struct usb_endpoint_descriptor  *ep_desc;
    int    rv;

    udev = interface_to_usbdev(intf);
    if(intf->cur_altsetting->desc.bInterfaceNumber != 0) return -ENODEV;

#ifdef  USE_PATCH
    rv = patch_add(intf);
    if(rv < 0) return rv;
#endif

    dev = kzalloc(sizeof(*dev),GFP_KERNEL); 
    if(!dev) return -ENOMEM;
    iface_desc = intf->cur_altsetting;
    for(int i = 0;i < iface_desc->desc.bNumEndpoints;i++){
        ep_desc = &(iface_desc->endpoint[i].desc);
        if(!dev->intr_ep && usb_endpoint_is_int_in(ep_desc)){
            dev->intr_ep = ep_desc;
            continue;
        }
        if(!dev->bulk_tx_ep && usb_endpoint_is_bulk_out(ep_desc)){
            dev->bulk_tx_ep = ep_desc;
            continue;
        }
        if(!dev->bulk_rx_ep && usb_endpoint_is_bulk_in(ep_desc)){
            dev->bulk_rx_ep = ep_desc;
            continue;
        }
    }
    if(!(dev->intr_ep && dev->bulk_tx_ep && dev->bulk_rx_ep)){
        rv =  -ENODEV;
        goto _kfree;
    }
    dev->udev = udev;
    dev->intf = intf;
    spin_lock_init(&dev->complete_spin);
	sema_init(&dev->limit_sem, WRITES_IN_FLIGHT);
    mutex_init(&dev->io_mutex);
    INIT_WORK(&dev->work,wb_work);
    INIT_WORK(&dev->sleep,wb_sleep);
    init_usb_anchor(&dev->submit);
	init_usb_anchor(&dev->isoc_anchor);
    init_completion(&dev->read_completion);
    dev->queue = newQueue();
    if(!dev->queue){
        rv = -ENOMEM;
        goto _kfree;
    }
    rv = usb_register_dev(intf,&wb_class);
    if(rv){
        warning("Not able to get a minor for this devoce.");
        usb_set_intfdata(intf,NULL);
        goto _delQueue;
    }
    dev->isoc = usb_ifnum_to_if(dev->udev,1);
    if(dev->isoc){
        rv = usb_driver_claim_interface(&usb_device,dev->isoc,dev);
        if(rv){
            warning("Not claim interface for this isoc");
            goto _deregister;
        }
    }
    usb_set_intfdata(intf,dev);
    return 0;
_deregister:
    usb_deregister_dev(intf,&wb_class);
_delQueue:
    delQueue(dev->queue,kfree);
_kfree:
    kfree(dev);
    return rv;
}


static void wb_disconnect(struct usb_interface *intf){
    struct usb_wb   *dev;
    //int             minor = intf->minor;
    dev = usb_get_intfdata(intf);
    if(intf->cur_altsetting->desc.bInterfaceNumber != 0) return;
    if(!dev) return;
#ifdef  USE_PATCH
    patch_remove(intf);
#endif
    usb_set_intfdata(intf,NULL);
    usb_deregister_dev(intf,&wb_class);
    mutex_lock(&dev->io_mutex);
    if(dev->isoc) usb_set_intfdata(dev->isoc,NULL);
    dev->intf = NULL;
    if(dev->isoc == intf)
        usb_driver_release_interface(&usb_device,dev->intf);
    else if(dev->isoc)
        usb_driver_release_interface(&usb_device,dev->isoc);
    mutex_unlock(&dev->io_mutex);
    delQueue(dev->queue,kfree);
    usb_kill_anchored_urbs(&dev->submit);
    kfree(dev);
}

static int __init wb_init(void){
    return usb_register(&usb_device);
}

static void __exit wb_exit(void){
    usb_deregister(&usb_device);
}
module_init(wb_init);
module_exit(wb_exit);

MODULE_AUTHOR("lzy <LuoZhongYao@gmail.com>");
MODULE_DESCRIPTION("Readtek Bluetooth USB driver version "VERSION);
MODULE_VERSION(VERSION);
MODULE_LICENSE("GPL");
