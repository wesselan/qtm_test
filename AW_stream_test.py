
import asyncio
#from scipy.spatial.transform import Rotation as R

import qtm

async def main():

    connection = await qtm.connect("127.0.0.1")

    if connection is None:
        print("Failed to connect :(")
        return
    else:
        print("Yay! Connected :)")

    async with qtm.TakeControl(connection,"password"):
        await connection.new()

    def on_packet(packet):
        # get and print packet number and bodies
        info, bodies = packet.get_6d() # what is packet.get_...
        print("Framenumber: {} - Body count: {}".format(
            packet.framenumber, info.body_count))

        # get and print 6DoF info
        for position, rotation in bodies:
            print("Pos: {}\nRot: {}".format(position, rotation))

        print(bodies)

    one_pack = await connection.get_current_frame(components=["6d"])

    on_packet(one_pack)
        
if __name__ == "__main__":
    # Run our asynchronous function until complete
    asyncio.run(main())
