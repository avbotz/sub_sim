#!/usr/bin/env python

"""
This node generates simulated data from the pinger and runs the MUSIC
algorithm to generate the angle to the pinger in the sim, publishing it
to a topic that simulator_node can read.

Authors: Nikhil Sunkad, Angad Bhargav
"""
import math
import rospy as ros
import numpy as np
import numpy.linalg as lin
from std_msgs.msg import Float64
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *

# Some global variables
nemo_coords = [0, 0, 0, 0, 0, 0]
pinger_coords = [34, 11]
spacing = 0.015
number_of_signal_sources = 1

X = 0
Y = 1
Z = 2
YAW = 3
PITCH = 4
ROLL = 5
 
def angle_difference(a1, a2):
	# Returns the difference between the two angles,
    # makes the difference between -180 and 180 degrees
    b1 = a1-a2
    if (abs(b1) > 180):
        if (a1 < a2):
            	a1 += 360
        else:
            a2 += 360
            b1 = a1 - a2
    return b1

def euler_from_quaternion(x, y, z, w):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = math.degrees(math.atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.degrees(math.asin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = math.degrees(math.atan2(t3, t4))

    return X, Y, Z

def update_nemo_coords():
    gms = ros.ServiceProxy('/gazebo/get_model_state', GetModelState)
    ros.wait_for_service('/gazebo/get_model_state')
    response = gms('nemo', 'world')

    # Update current nemo position
    nemo_coords[X] = response.pose.position.x
    nemo_coords[Y] = -response.pose.position.y
    nemo_coords[Z] = -response.pose.position.z

    # Convert quaternions to North-East-Down Euler angles
    roll, pitch, yaw = euler_from_quaternion(
        response.pose.orientation.x,
        response.pose.orientation.y,
        response.pose.orientation.z,
        response.pose.orientation.w
    )

    nemo_coords[YAW] = -yaw
    nemo_coords[PITCH] = -pitch
    nemo_coords[ROLL] = roll

def get_real_angle_to_pinger():
    update_nemo_coords()

    real_yaw = math.degrees(math.atan2(
        pinger_coords[X] - nemo_coords[X], 
        pinger_coords[Y] - nemo_coords[Y]
    ))
    print(nemo_coords[YAW])

    return angle_difference(real_yaw, nemo_coords[YAW])

def gen_steering_vectors(M, x, y, theta_values):
    number_of_angles = theta_values.size
    steering_vectors = np.zeros((M, number_of_angles), dtype=complex)
    for i in range(number_of_angles):
        steering_vectors[:, i] = np.exp(1j * 2 * np.pi * ( x * np.cos(np.deg2rad(theta_values[i])) + y * np.sin(np.deg2rad(theta_values[i]))))

    return steering_vectors

def MUSIC(covariance_matrix, steering_vectors, angle_resolution = 1):
    spectrum = np.zeros(np.size(steering_vectors, 1),dtype=complex)
    M = np.size(covariance_matrix, 0)

    # Determine eigenvectors and eigenvalues of covariance_matrix
    eigenvalues, eigenvectors = lin.eig(np.array(covariance_matrix))

    # Sorting the eigenvectors by order of ascending eigenvalues
    sorted_eigenvectors = []
    for i in range(M):
        sorted_eigenvectors.append([np.abs(eigenvalues[i]), eigenvectors[:,i]])
    sorted_eigenvectors = sorted(sorted_eigenvectors, key=lambda sorted_eigenvectors: sorted_eigenvectors[0], reverse=False)

    # Generate noise subspace matrix; isolate M-D eigenvalues
    noise_dimension = M - number_of_signal_sources
    E = np.zeros((M,noise_dimension),dtype=complex)
    for i in range(noise_dimension):
        E[:,i] = sorted_eigenvectors[i][1]

    E = np.matrix(E)

    theta_index = 0
    for i in range(np.size(steering_vectors, 1)):
        vector = np.matrix(steering_vectors[:, i])
        vector = np.matrix(vector).getT()
        spectrum[theta_index] = 1 / np.abs(vector.getH()*(E*E.getH())*vector)
        theta_index += 1

    return spectrum

def main():
    ros.init_node("hydrophone_node")
    pub = ros.Publisher('angle_to_pinger', Float64, queue_size=10)

    number_of_hydrophones = 4
    number_of_samples = 286000
    incident_angles = np.arange(0, 180.1, 0.1)

    while not ros.is_shutdown():
        # Set angle of incidence
        theta = get_real_angle_to_pinger()

        # Generate scanning vectors with the general purpose function
        x = np.arange(0, number_of_hydrophones, 1) * spacing # x coordinates
        y = np.zeros(number_of_hydrophones) # y coordinates
        steering_vectors = gen_steering_vectors(number_of_hydrophones, x, y, incident_angles)

        # Array response vectors of the test signal
        A = np.exp(np.arange(0, number_of_hydrophones, 1) * 1j * 2 * np.pi * spacing * np.cos(np.deg2rad(theta)))

        # Generate multichannel test signal
        soi = np.abs(np.random.normal(5, 10, number_of_samples))  # Signal of Interest
        soi_matrix = np.outer(soi, A).T

        # Generate multichannel uncorrelated noise
        noise = np.random.normal(0, np.sqrt(10**-1), (number_of_hydrophones, number_of_samples))

        # Create received signal array
        rec_signal = np.matrix(soi_matrix + noise)

        # Estimating the spatial correlation matrix
        covariance_matrix = np.dot(rec_signal, rec_signal.getH()) / number_of_samples

        spectrum = MUSIC(covariance_matrix, steering_vectors)

        angle_to_pinger = np.argmax(spectrum) * 0.1

        # Publish the calculated angle
        pub.publish(Float64(angle_to_pinger))

if __name__ == '__main__':
    main()
