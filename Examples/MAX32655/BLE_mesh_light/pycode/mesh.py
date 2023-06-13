import asyncio
import secrets
from contextlib import suppress

from docopt import docopt

from bluetooth_mesh.application import Application, Element
from bluetooth_mesh.crypto import ApplicationKey, DeviceKey, NetworkKey
from bluetooth_mesh.messages.config import GATTNamespaceDescriptor, StatusCode
from bluetooth_mesh.models import ConfigClient, HealthClient


class MainElement(Element):
    LOCATION = GATTNamespaceDescriptor.MAIN
    MODELS = [
        ConfigClient,
        HealthClient,
    ]


class SampleApplication(Application):
    COMPANY_ID = 0x0136  # Silvair
    PRODUCT_ID = 1
    VERSION_ID = 1
    ELEMENTS = {
        0: MainElement,
    }
    CRPL = 32768
    PATH = "/com/silvair/sample"

    @property
    def dev_key(self):
        return DeviceKey(secrets.token_bytes(16))

    @property
    def net_key(self):
        return 0, NetworkKey(secrets.token_bytes(16))

    @property
    def app_keys(self):
        return {0: ApplicationKey(secrets.token_bytes(16))}

    async def configure(self):
        client = self.elements[0][ConfigClient]

        status = await client.add_app_key(
            self.addr, net_index=0,
            app_key_index=0,
            net_key_index=0,
            app_key=self.app_keys[0]
        )

        assert status == StatusCode.SUCCESS, \
            'Cannot add application key: %s' % status

        status = await client.bind_app_key(
            self.addr, net_index=0,
            element_address=self.addr,
            app_key_index=0,
            model=HealthClient
        )

        assert status == StatusCode.SUCCESS, \
            'Cannot bind application key: %s' % status

    async def run(self, addr):
        async with self:
            await self.connect(addr=addr, iv_index=5)
            await self.configure()

            client = self.elements[0][HealthClient]

            for node in [0x0001, 0x0002, 0x0003]:
                await client.attention(node, app_index=0, attention=3)


def main():
    doc = """
    Sample Application

    Usage:
        sample --address <address>
        meshcli -h | --help | --version

    Options:
        -a --address <address>          Local node unicast address
        -h --help                       Show this help message and exit
        --version                       Show version and exit
    """

    arguments = docopt(doc, version='1.0')

    addr = arguments['--address']
    addr = int(addr, 16 if addr.startswith('0x') else 10)

    loop = asyncio.get_event_loop()
    app = SampleApplication(loop)

    with suppress(KeyboardInterrupt):
        loop.run_until_complete(app.run(addr))


if __name__ == '__main__':
    main()