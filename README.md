# Alsa
Drivers alsa for Digigram audio cards :
- LX : LX-MADI, LX-IP, LX-IP-MAD


add in your headers -> include/linux/pci_ids.h

#define PCIEX_SUBDEVICE_ID_DIGIGRAM_LXMADI_SERIAL_SUBSYSTEM     0xca21
#define PCIEX_SUBDEVICE_ID_DIGIGRAM_LXIP_SERIAL_SUBSYSTEM       0xc821
#define PCIEX_SUBDEVICE_ID_DIGIGRAM_LXIP_MADI_SERIAL_SUBSYSTEM  0xcc21


