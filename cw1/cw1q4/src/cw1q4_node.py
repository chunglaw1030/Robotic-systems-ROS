#!/usr/bin/env python

import rospy
import numpy as np

# TODO: Include all the required service classes
# your code starts here -----------------------------
from cw1q4_srv.srv import quat2rodrigues, quat2rodriguesResponse,
quat2rodriguesRequest
# your code ends here -------------------------------
from cw1q4_srv.srv import quat2zyx, quat2zyxResponse, quat2zyxRequest



def convert_quat2zyx(request):
    # TODO complete the function
    """Callback ROS service function to convert quaternion to Euler z-y-x representation

    Args:
        request (quat2zyxRequest): cw1q4_srv service message, containing
        the quaternion you need to convert.

    Returns:
        quat2zyxResponse: cw1q4_srv service response, in which 
        you store the requested euler angles 
    """
    assert isinstance(request, quat2zyxRequest)

    # Your code starts here ----------------------------

	qx = request.q.x
	qy = request.q.y
	qz = request.q.z
	qw = request.q.w

	response = quat2zyxResponse()	

	response.x = np.arctan2(2*(qw*qx + qy*qz),1-2*(qx*qx+qy*qy)

	y1 = 2*(qw*qy - qz*qx)
	y1 = 1 if y1 > 1 else y1
	y1 = -1 if y1< -1 else y1
	response.y = np.arcsin(y1)

	response.z = np.arctan2(2*(qw*qz + qx*qy), 1-2*(qy*qy+qz*qz)

		
    # Your code ends here ------------------------------

    assert isinstance(response, quat2zyxResponse)
    return response


def convert_quat2rodrigues(request):
    # TODO complete the function

    """Callback ROS service function to convert quaternion to rodrigues representation
    
    Args:
        request (quat2rodriguesRequest): cw1q4_srv service message, containing
        the quaternion you need to convert

    Returns:
        quat2rodriguesResponse: cw1q4_srv service response, in which 
        you store the requested rodrigues representation 
    """
    assert isinstance(request, quat2rodriguesRequest)

    # Your code starts here ----------------------------
	qx = request.q.x
	qy = request.q.y
	qz = request.q.z
	qw = request.q.w
	
	response = quat2rodriguesResponse()	

	theta = np.arccos(qw)*2

	sx = qx/np.sin(theta/2)
	
	sy = qy/np.sin(theta/2)

	sz = qz/np.sin(theta/2)

	response.x = np.tan(thetaa/2)*sx
	response.y = np.tan(thetaa/2)*sy
	response.z = np.tan(thetaa/2)*sz
	



    # Your code ends here ------------------------------

    assert isinstance(response, quat2rodriguesResponse)
    return response

def rotation_converter():
    rospy.init_node('rotation_converter')

    #Initialise the services
    rospy.Service('quat2rodrigues', quat2rodrigues, convert_quat2rodrigues)
    rospy.Service('quat2zyx', quat2zyx, convert_quat2zyx)

    rospy.spin()


if __name__ == "__main__":
    rotation_converter()
