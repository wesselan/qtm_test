import asyncio
import numpy as np
from scipy.spatial.transform import Rotation as rotat

import qtm


async def main():
    connection = await qtm.connect("127.0.0.1")

    if connection is None:
        print("Failed to connect :(")
        return
    else:
        print("Yay! Connected :)")

    # close qtm files and start realtime camera feed
    async with qtm.TakeControl(connection, "password"):
        await connection.new()

    def on_packet(packet):
        # get and print packet number and bodies
        info, bodies = packet.get_6d()  # what is packet.get_...
        print("Frame number: {} - Body count: {}".format(
            packet.framenumber, info.body_count))

        # get and print 6DoF info
        for position, rotation in bodies:
            # print("Pos: {}\nRot: {}".format(position, rotation))

            # get rot matrix
            rotation_matrix = np.array(rotation.matrix)
            rotation_matrix.shape = (3, 3)
            print(rotation_matrix)

            # turn rot matrix into quaternion
            rotation_matrix = rotat.from_matrix(rotation_matrix)
            print(rotation_matrix.as_quat())

            # add pos and quaternion to mavlink message
            return rotation_matrix.as_quat()

    one_pack = await connection.get_current_frame(components=["6d"])

    on_packet(one_pack)


if __name__ == "__main__":
    # Run our asynchronous function until complete
    asyncio.run(main())
