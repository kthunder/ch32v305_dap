"""USB 设备管理器 — 封装设备查找、端点初始化等底层操作"""

from typing import Optional, Any
import usb.core
import usb.util


class UsbDeviceManager:
    """USB设备管理器 - 封装USB设备操作，提供类型安全的端点访问"""

    def __init__(self, dev: usb.core.Device):
        super().__init__()
        self._dev: usb.core.Device = dev
        self._ep_out: Optional[Any] = None
        self._ep_in: Optional[Any] = None
        
        cfg = dev.get_active_configuration()
        for interface in cfg:
            # print(f"  Interface {interface.bInterfaceNumber}: class=0x{interface.bInterfaceClass:02X}, endpoints={interface.bNumEndpoints}")
            # for ep in interface:
            #     print(f"    EP 0x{ep.bEndpointAddress:02X}: bmAttributes=0x{ep.bmAttributes:02X}")
            if interface.bInterfaceClass == 0xFF:
                self._ep_out = usb.util.find_descriptor(
                    interface,
                    custom_match=lambda e: (e.bEndpointAddress & 0x80 == 0) and ((e.bmAttributes & 0x03) == 0x02)
                )
                self._ep_in = usb.util.find_descriptor(
                    interface,
                    custom_match=lambda e: (e.bEndpointAddress & 0x80 != 0) and ((e.bmAttributes & 0x03) == 0x02)
                )
                # print(f"    -> ep_out={self._ep_out}, ep_in={self._ep_in}")
                if self._ep_out is not None:
                    break

    @staticmethod
    def get_dev_list() -> list[usb.core.Device]:
        dev_list = []
        if res := usb.core.find(find_all=True, idVendor=0x0D28, idProduct=0x0204):
            for dev in res:
                if isinstance(dev, usb.core.Device):
                    dev.set_configuration()
                    cfg = dev.get_active_configuration()
                    # 查找自定义接口
                    for interface in cfg:
                        if interface.bInterfaceClass == 0xFF:
                            intf = interface
                            dev_list.append(dev)
        return dev_list



    def send(self, data: bytes) -> bool:
        return self._dev.write(self._ep_out, data)
        # return True

    def recv(self, n: int) -> bytes:
        return bytes(self._dev.read(self._ep_in, n))
        # return bytes(0x100)

if __name__ == "__main__":
    devs = UsbDeviceManager.get_dev_list()
    print(devs)
    usb_manager = UsbDeviceManager(devs[0])
    manufacturer = usb.util.get_string(devs[0], devs[0].iManufacturer)  # type: ignore[union-attr]
    product = usb.util.get_string(devs[0], devs[0].iProduct)  # type: ignore[union-attr]
    print(manufacturer)
    print(product)
    usb_manager.send(bytes([0x9E]))
