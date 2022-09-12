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
            """
            From SciPy - https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.as_quat.html#scipy.spatial.transform.Rotation.as_quat
            Represent as quaternions.
             Rotations in 3 dimensions can be represented using unit norm quaternions [1]. The mapping from quaternions 
             to rotations is two-to-one, i.e. quaternions q and -q, where -q simply reverses the sign of each component, 
             represent the same spatial rotation. The returned value is in scalar-last (x, y, z, w) format.  
            """
            """
            From MAVSDK - https://github.com/mavlink/MAVSDK-Python/blob/main/mavsdk/mocap.py
             Quaternion type.
             All rotations and axis systems follow the right-hand rule.
             The Hamilton quaternion product definition is used.
             A zero-rotation quaternion is represented by (1,0,0,0).
             The quaternion could also be written as w + xi + yj + zk.
             For more info see: https://en.wikipedia.org/wiki/Quaternion
             Parameters
             ----------
             w : float
                  Quaternion entry 0, also denoted as a
             x : float
                  Quaternion entry 1, also denoted as b
             y : float
                  Quaternion entry 2, also denoted as c
             z : float
                  Quaternion entry 3, also denoted as d
             """
            return rotation_matrix.as_quat()

    one_pack = await connection.get_current_frame(components=["6d"])

    on_packet(one_pack)


if __name__ == "__main__":
    # Run our asynchronous function until complete
    asyncio.run(main())
