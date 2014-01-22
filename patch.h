#ifndef __RTK_PATCH_H__
#define __RTK_PATCH_H__
#include    <linux/usb.h>
int download_patch(struct usb_interface *intf);
int patch_add(struct usb_interface *intf);
void patch_remove(struct usb_interface *intf);

#endif
