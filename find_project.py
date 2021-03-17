import numpy as np
import cv2
import os
import copy
from filterpy.kalman import KalmanFilter
from numpy.linalg import inv
import matplotlib.pyplot as plt
np.set_printoptions(suppress=True) 

g = 9.81
dt = 1 / 8
dist = 150
c_size = 500

def find(image, x_last, y_last, base):
    # get frame ready
    frame = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    frame = cv2.GaussianBlur(frame, (11,11), 0)

    # calculate difference between og frame and current one
    delta = cv2.absdiff(base, frame)
    # threshold movement
    thresh = cv2.threshold(delta, 10, 255, cv2.THRESH_BINARY)[1]
    # find contours in threshold
    contours, hierarchy = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) != 0:
        # for c in contours:
        #     print(cv2.contourArea(c))
        # print()
        for c in contours:
            # min contour size
            if cv2.contourArea(c) > c_size:
                # compute the center of the contour
                M = cv2.moments(c)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                # if it's close to the previous one, use it
                if x_last == -1 and y_last == -1:
                    return True, cX, cY
                if np.abs(cX-x_last) < dist and np.abs(cY-y_last) < dist:
                    return True, cX, cY

    return False, x_last, y_last
                    
def iterate(file_l, file_r, start, length):
    # set everything up
    num_found = 0
    num_lost  = 0
    cap_l = cv2.VideoCapture(file_l)
    cap_r = cv2.VideoCapture(file_r)
    x_l, y_l = -1, -1
    x_r, y_r = -1, -1
    past_traj = []

    # get to the right spot in the video
    for _ in range(start-1):
        cap_l.read()
        cap_r.read()
    base_l = cv2.cvtColor(cap_l.read()[1], cv2.COLOR_BGR2GRAY)
    base_r = cv2.cvtColor(cap_r.read()[1], cv2.COLOR_BGR2GRAY)
    h, w = base_l.shape

    # iterate through our frames
    for i in range(length):
        ret, image_l = cap_l.read()
        ret, image_r = cap_r.read()

        found_r, x_r, y_r = find(image_r, x_r, y_r, base_r)
        found_l, x_l, y_l = find(image_l, x_l, y_l, base_l)
        image_l = cv2.circle(image_l, (x_l, y_l), 4, (0,0,255), 2)
        image_r = cv2.circle(image_r, (x_r, y_r), 4, (0,0,255), 2)

    
        # in the case we lost the ball
        if not found_l or not found_r:
            num_lost += 1
            # if we've lost it for a while, assume it's left
            if num_lost > 5:
                if 'f' in locals():
                    del f
                num_found = 0
                x_l, y_l = -1, -1
                x_r, y_r = -1, -1

        # if we found the ball in both frames
        if found_l and found_r:
            # if it's the first time, save it for later
            if num_found == 0:
                loc_1 = pts_to_3d(x_l, y_l, x_r, y_r)
                past_traj.append(threed_to_pts(loc_1))
                print("Location in left:\t", x_l, y_l)
                print("Location in right:\t", x_r, y_r)
                print("3D Location:\t", loc_1)
            # if we've found it before, update
            else:
                loc = pts_to_3d(x_l, y_l, x_r, y_r)
                # make filter on 2nd one
                if num_found == 1:
                    start = np.append( loc, (loc-loc_1)/dt )
                    f = make_filter(start)
                # or update if it's later
                else:
                    f.update(loc)
                print("Location in left:\t", x_l, y_l)
                print("Location in right:\t", x_r, y_r)
                print("3D Location:\t", loc)
            num_lost = 0
            num_found += 1

        if 'f' in locals():
            past_traj.append(threed_to_pts(f.x[:3]))
            print(f.x)

            # get predictions and covariances in 3D
            p_state_3d, p_cov = predict(f, 10)
            temp = [cov_to_ellipse(p) for p in p_cov]
            p_cov_length = np.array([t[0] for t in temp])
            p_cov_theta = np.array([t[1] for t in temp])

            # Project them to the image
            p_state = np.array([threed_to_pts(p) for p in p_state_3d], dtype=np.int)
            p_cov = np.array([threed_to_pts(s+c)-i for s, i, c in zip(p_state_3d, p_state, p_cov_length)])

            # visualize
            for state, ellipse, theta in zip(p_state, p_cov, p_cov_theta):
                try:
                    image_l = cv2.ellipse(image_l, tuple(state), tuple(ellipse), float(theta), 0.0, 360.0, (0,0,255), 1)
                except:
                    print(f"Broke with {ellipse}, {state}, likely offscreen")
            image_l = cv2.polylines(image_l, [p_state], False, (0,255,0), 2)
            image_l = cv2.polylines(image_l, [np.array(past_traj)], False, (255,0,0), 2)
            # print()
            # get ready for next step
            f.predict(g)

        cv2.imshow(f'find', cv2.hconcat([image_l, image_r]))
        if cv2.waitKey() == ord('q'):
            break

    # plt.show()

def predict(f, steps):
    state = [f.x[:3]]
    cov   = [f.P[:2,:2]]
    f = copy.deepcopy(f)
    for i in range(steps):
        f.predict(g)
        state.append(f.x[:3])
        cov.append(f.P[:2,:2])

    return np.array(state), np.array(cov)

def make_filter(start):
    f = KalmanFilter(dim_x=6, dim_z=3)
    f.x = start

    f.F = np.eye(6)
    f.F[:3,3:6] = np.eye(3)*dt
    f.B = np.array([0, dt**2/2, 0, 0, dt, 0])
    g = -9.81

    f.H = np.zeros((3,6))
    f.H[:,:3] = np.eye(3)

    f.Q *= 0.5
    f.R *= 2
    f.P *= .1

    return f

def pts_to_3d(x_l, y_l, x_r, y_r):
    # undistort points
    pts_l = cv2.undistortPoints(np.array([[x_l, y_l]], dtype=float), mtx_l, dist_l, R=R_l, P=P_l)
    pts_r = cv2.undistortPoints(np.array([[x_r, y_r]], dtype=float), mtx_r, dist_r, R=R_r, P=P_r)

    # perspective transform
    disp = ( pts_l[:,:,0] - pts_r[:,:,0] ).reshape((len(pts_l), 1, 1))
    pts_l = np.concatenate((pts_l, disp), axis=-1)

    return cv2.perspectiveTransform(pts_l, Q).squeeze()

def threed_to_pts(loc):
    # undo perspectiveTransform
    pts = cv2.perspectiveTransform(loc.reshape((1,1,3)), np.linalg.inv(Q)).squeeze()[:2]

    # undo undistortPoints
    x, y = (pts - P_l[:2,2]) / P_l[[0,1], [0,1]]
    x,y,z = inv(R_l)@np.array([x,y,1])
    x, y = x/z, y/z
    # this is an approximation for redistorting, but is good enough for our purposes
    pts = cv2.undistortPoints(np.array([[x,y]], dtype=float), np.eye(3), -dist_l, R=np.eye(3), P=mtx_l)
    return np.rint(pts.squeeze()).astype('int')


def cov_to_ellipse(cov, n_sig=3):
    lam, v = np.linalg.eigh(cov)

    # get angle
    if np.allclose(v[1,0], 0):
        theta = 90
    else:
        theta = np.arctan(v[0,0]/v[1,0])*180/np.pi

    # get length of major ellipses
    a, b = n_sig*np.sqrt(lam)

    return [a, b, 0], theta

if __name__ == "__main__":
    params = np.load('third/params.npz')
    locals().update(params)
    iterate('third/first_L.avi',
            'third/first_R.avi',
            520, 80)

    # params = np.load('trajectory/params.npz')
    # locals().update(params)
    # iterate('trajectory/BaseBall_Pitch_L.avi',
    #         'trajectory/BaseBall_Pitch_R.avi',
    #         451, 50)
