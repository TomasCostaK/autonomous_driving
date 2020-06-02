'''
Ref: https://github.com/charlesq34/frustum-pointnets/blob/master/kitti/kitti_util.py
'''
import numpy as np
import cv2



def project_to_image(pts_3d, P):
    ''' Project 3d points to image plane.

    Usage: pts_2d = projectToImage(pts_3d, P)
      input: pts_3d: nx3 matrix
             P:      3x4 projection matrix
      output: pts_2d: nx2 matrix

      P(3x4) dot pts_3d_extended(4xn) = projected_pts_2d(3xn)
      => normalize projected_pts_2d(2xn)

      <=> pts_3d_extended(nx4) dot P'(4x3) = projected_pts_2d(nx3)
          => normalize projected_pts_2d(nx2)
    '''
    n = pts_3d.shape[0]
    pts_3d_extend = np.hstack((pts_3d, np.ones((n,1))))
    print(('pts_3d_extend shape: ', pts_3d_extend.shape))
    pts_2d = np.dot(pts_3d_extend, np.transpose(P)) # nx3
    pts_2d[:,0] /= pts_2d[:,2]
    pts_2d[:,1] /= pts_2d[:,2]
    return pts_2d[:,0:2]

def roty(t):
    ''' Rotation about the y-axis. '''
    c = np.cos(t)
    s = np.sin(t)
    return np.array([[c,  0,  s],
                     [0,  1,  0],
                     [-s, 0,  c]])

def project_rect_to_velo(pts_3d_rect, R0, C2V_mat):
        ''' Input: nx3 points in rect camera coord.
            Output: nx3 points in velodyne coord.
        ''' 
        pts_3d_ref = project_rect_to_ref(pts_3d_rect, R0)
        return project_ref_to_velo(pts_3d_ref, C2V_mat)

def project_rect_to_ref(pts_3d_rect, R0):
        ''' Input and Output are nx3 points '''
        return np.transpose(np.dot(np.linalg.inv(R0), np.transpose(pts_3d_rect)))

def project_ref_to_velo(pts_3d_ref, C2V_mat):
        pts_3d_ref = cart2hom(pts_3d_ref) # nx4
        return np.dot(pts_3d_ref, np.transpose(C2V_mat))

def cart2hom(pts_3d):
        ''' Input: nx3 points in Cartesian
            Oupput: nx4 points in Homogeneous by pending 1
        '''
        n = pts_3d.shape[0]
        pts_3d_hom = np.hstack((pts_3d, np.ones((n,1))))
        return pts_3d_hom

'''
l, w, h: 3d bouding box dimenions
ry: object rotation around up axis
t: object position
P: projection matrix (p2)
'''
def compute_box_3d(l, w, h, ry, t, P, R0, C2V_mat): #obj, P):
    ''' Takes an object and a projection matrix (P) and projects the 3d
        bounding box into the image plane.
        Returns:
            corners_2d: (8,2) array in left image coord.
            corners_3d: (8,3) array in in rect camera coord.
    '''
    # compute rotational matrix around yaw axis
    R = roty(ry)    
    
    # 3d bounding box corners
    x_corners = [l/2,l/2,-l/2,-l/2,l/2,l/2,-l/2,-l/2];
    y_corners = [0,0,0,0,-h,-h,-h,-h];
    z_corners = [w/2,-w/2,-w/2,w/2,w/2,-w/2,-w/2,w/2];
    
    # rotate and translate 3d bounding box
    corners_3d = np.dot(R, np.vstack([x_corners,y_corners,z_corners]))

    print("corners: " + str(corners_3d))

    print("object center: " + str(t))

    corners_3d[0,:] = corners_3d[0,:] + t[0];
    corners_3d[1,:] = corners_3d[1,:] + t[1];
    corners_3d[2,:] = corners_3d[2,:] + t[2];

    print("corners >>: " + str(corners_3d))

    # project the 3d bounding box into the image plane
    corners_2d = project_to_image(np.transpose(corners_3d), P);

    corners_3d = np.transpose(corners_3d)

    box3d_pts_3d_velo = project_rect_to_velo(corners_3d, R0, C2V_mat)
    
    return corners_2d, box3d_pts_3d_velo

def draw_projected_box3d(image, qs, color=(255,255,255), thickness=2):
    ''' Draw 3d bounding box in image
        qs: (8,3) array of vertices for the 3d box in following order:
            1 -------- 0
           /|         /|
          2 -------- 3 .
          | |        | |
          . 5 -------- 4
          |/         |/
          6 -------- 7
    '''
    print("qs: " + str(qs))

    qs = qs.astype(np.int32)
    for k in range(0,4):
       # Ref: http://docs.enthought.com/mayavi/mayavi/auto/mlab_helper_functions.html
       i,j=k,(k+1)%4
       # use LINE_AA for opencv3
       cv2.line(image, (qs[i,0],qs[i,1]), (qs[j,0],qs[j,1]), color, thickness)

       i,j=k+4,(k+1)%4 + 4
       cv2.line(image, (qs[i,0],qs[i,1]), (qs[j,0],qs[j,1]), color, thickness)

       i,j=k,k+4
       cv2.line(image, (qs[i,0],qs[i,1]), (qs[j,0],qs[j,1]), color, thickness)
    return image