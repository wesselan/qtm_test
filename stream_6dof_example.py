"""
    Streaming 6Dof from QTM
"""

import asyncio
import xml.etree.ElementTree as ET
import pkg_resources
import numpy as np
from scipy.spatial.transform import Rotation as R
# import mavsdk


import qtm

QTM_FILE = pkg_resources.resource_filename("qtm", "data/Demo.qtm")


def create_body_index(xml_string):
    """ Extract a name to index dictionary from 6dof settings xml """
    xml = ET.fromstring(xml_string)

    body_to_index = {}
    for index, body in enumerate(xml.findall("*/Body/Name")):
        body_to_index[body.text.strip()] = index

    return body_to_index


async def main():

    # Connect to qtm
    connection = await qtm.connect("127.0.0.1")

    # Connection failed?
    if connection is None:
        print("Failed to connect")
        return

    # Take control of qtm, context manager will automatically release control after scope end
    async with qtm.TakeControl(connection, "password"):

        realtime = False

        if realtime:
            # Start new realtime
            await connection.new()
        else:
            # Load qtm file
            await connection.load(QTM_FILE)

            # start rtfromfile
            await connection.start(rtfromfile=True)

    # Get 6dof settings from qtm
    xml_string = await connection.get_parameters(parameters=["6d"])
    body_index = create_body_index(xml_string)

    wanted_body = "L-frame"

    def on_packet(packet):
        info, bodies = packet.get_6d()
        time_var = packet.get_timecode()
        print(time_var)
        print(
            "FrameNumber: {} - Body count: {}".format(
                packet.framenumber, info.body_count
            )
        )

        if wanted_body is not None and wanted_body in body_index:
            # Extract one specific body
            wanted_index = body_index[wanted_body]
            position, rotation = bodies[wanted_index]
            # print("{} - Pos: {} - Rot: {}".format(wanted_body, position, rotation))
            # print(bodies[wanted_index][1].matrix[0])
            rotation_matrix = np.array(rotation.matrix)
            # this is to go from vector to matrix
            # <Data_orientation R11 R12 R13 R21 R22 R23 R31 R32 R33 Relative_body>
            # https://docs.qualisys.com/qtm-rt-protocol/#6d-xml-parameters
            rotation_matrix.shape = (3, 3)
            print(rotation_matrix)
            rotation_matrix = R.from_matrix(rotation_matrix)
            scipy_quaternion = rotation_matrix.as_quat()
            reorder_idx = [1, 2, 3, 0]
            mavsdk_quaternion = scipy_quaternion[reorder_idx]
            print(scipy_quaternion)
            print(mavsdk_quaternion)
            """
            WIP: Creating and sending the mavlink message
            mavsdk.mocap.AttitudePositionMocap(time_usec, mavsdk_quaternion, position, pose_covariance)
            """
        else:
            # Print all bodies
            for position, rotation in bodies:
                print("Pos: {} - Rot: {}".format(position, rotation))

    # Start streaming frames
    await connection.stream_frames(frames="frequency:1", components=["6d", "timecode"], on_packet=on_packet)

    # Wait asynchronously 5 seconds
    await asyncio.sleep(5)

    # Stop streaming
    await connection.stream_frames_stop()


if __name__ == "__main__":
    # Run our asynchronous function until complete
    asyncio.get_event_loop().run_until_complete(main())
