from nuscenes.nuscenes import NuScenes
import numpy as np
from scipy.spatial.transform import Rotation as R
nusc = NuScenes(version='v1.0-mini', dataroot='/home/nuscenes/data/sets/nuscenes', verbose=False)
from nuscenes.utils.data_classes import LidarPointCloud
#####################################################################
class lidar:
    def __init__(self):
        super().__init__()

    def get_lidar_cloud(self,scene_number,sample_number):  
        """
        This method loads Lidar data from a PointCloudData-File. (e.g '/data/sets/nuscenes/samples/RADAR..').
		based on the scene and sample number entered by the user

        Parameters
        ----------
        scene_number : int
            The scene number (see nuscenes basic tutorial)	
		sample_number : int
            The sample number (see nuscenes basic tutorial)

        Returns
        -------
        x,y,z : each is np.ndarray(1, n)
            LidarPoint cloud as points of (x,y,z).

        """  
        base_sample_token=self.get_sample_token(scene_number,sample_number)
        my_sample = nusc.get('sample', base_sample_token)
        nusc.get('sample_data',my_sample['data']['LIDAR_TOP'])['filename']
        file_path='/home/nuscenes/data/sets/nuscenes/'+ nusc.get('sample_data',my_sample['data']['LIDAR_TOP'])['filename']
        lidar_object= LidarPointCloud.from_file(file_path)
        lidar_points=lidar_object.points
        # will return x,y,z,intensity
        x=lidar_points[0].reshape((-1,1))
        y=lidar_points[1].reshape((-1,1))
        z=lidar_points[2].reshape((-1,1))
        return x,y,z
#####################################################################
    def get_ego_pose(self,scene_number,sample_number) :
        """
        This method get the ego pose at a certain sample

        Parameters
        ----------
        scene_number : int
            The scene number (see nuscenes basic tutorial)	
		sample_number : int
            The sample number (see nuscenes basic tutorial)
        Returns
        -------
        ego_pose: dictionary
            from which we can obtain the translation and quaternion of the ego pose (e.g : ego_pose['rotation'],  ego_pose['translation'])
        """
        sensor='LIDAR_TOP'
        token=self.get_sample_token(scene_number,sample_number)
        my_sample= nusc.get('sample',token)
        sensor_data = nusc.get('sample_data', my_sample['data'][sensor])    
        ego_token=sensor_data['ego_pose_token'] 
        ego_pose = nusc.get('ego_pose', ego_token)
        return ego_pose
 #######################################################################
    def get_sample_token(self,scene_number,sample_number):
    
        """
        This method get the sample token 

        Parameters
        ----------
        scene_number : int
            The scene number (see nuscenes basic tutorial)	
		sample_number : int
            The sample number (see nuscenes basic tutorial)
        Returns
        -------
        my_sample_token: str
            sample token which can be useful in other methods
        """
        samples_num=nusc.scene[scene_number]['nbr_samples']
        my_scene=nusc.scene[scene_number]
        first_sample_token=my_scene['first_sample_token']
		#first_sample_token = my_scene['first_sample_token']
        base_sample = nusc.get('sample', first_sample_token)
        if sample_number==0:
            my_sample_token=first_sample_token
        elif sample_number == samples_num:
            my_sample_token = my_scene['last_sample_token']
        else:
            for i in range(sample_number):
                my_sample_token=base_sample['next']
                base_sample=nusc.get('sample', my_sample_token)
        return my_sample_token
#####################################################################
    def getT(self,Q,t_sensor):
        """
        This method convert  ego pose translation and quaternion matricesto transformation matrix 

        Parameters
        ----------
        Q: list
            quaternion of the ego pose 
		t_sensor : list
            translation vector of the ego pose
        Returns
        -------
        T: np.ndarray
            transformation matrix
        
        """
        Q = np.array([Q[1],Q[2],Q[3],Q[0]])
        H = np.array([0,0 ,0 ,1])
        H = np.reshape(H,[1,4])
        t_sensor = np.reshape(t_sensor,[3,1])
        r = R.from_quat(Q)
        r = r.as_matrix()
        T = np.concatenate((r,t_sensor),1)
        T = np.concatenate((T,H),0)
        return T
 ##################################################################       
    def get_lidar_sample(self,scene_number,sample_number):
        """
        This method convert the lidar points cloud  from one sample  to world frame

        Parameters
        ----------
        scene_number : int
            The scene number (see nuscenes basic tutorial)	
		sample_number : int
            The sample number (see nuscenes basic tutorial)
        Returns
        -------
        x,y : each is np.ndarray 
            lidar x,y point in the world frame. notice we got rid of the z component
        """
        sensor='LIDAR_TOP'
        x,y,z=self.get_lidar_cloud(scene_number,sample_number)
        ego_pose = self.get_ego_pose(scene_number,sample_number)
        q = ego_pose['rotation']
        t = ego_pose['translation']
        T_ego_to_world = self.getT(q,t) 
        ones_homogeneous = np.ones_like(x)
        token=self.get_sample_token(scene_number,sample_number)
        my_sample= nusc.get('sample', token)
        sensor_data= nusc.get('sample_data', my_sample['data'][sensor])
        qua=nusc.get('calibrated_sensor', sensor_data['calibrated_sensor_token'])['rotation']
        trans =nusc.get('calibrated_sensor', sensor_data['calibrated_sensor_token'])['translation']
        #trans= [0.943713, 0.0, 1.84023]
        #qua=[0.7077955119163518,-0.006492242056004365,0.010646214713995808,-0.7063073142877817]
        T_lidar_to_ego = self.getT(qua,trans)
		#we arrange them into a matrix the ones vector is to make each row vector a homogeneous 
        XYZ_lidar = np.concatenate((x,y,z,ones_homogeneous),1)
        XYZ_world = np.array([0,0,0,0])
        for i in range(len(x)):
            XYZ_world  = np.hstack((XYZ_world,T_ego_to_world.dot(T_lidar_to_ego).dot(XYZ_lidar[i][:])))
        XYZ_world = np.reshape(XYZ_world,(-1,4))
        XYZ_world = np.delete(XYZ_world,0,0)
		#we need to delete the 1's  homogeneous  
        XYZ_world = np.delete(XYZ_world,3,1)
        x=XYZ_world[:,0]
        y=XYZ_world[:,1]
        z=XYZ_world[:,2]
        return x,y 
#####################################################################
    def get_lidar_pose(self,scene_number,sample_number):
        """
        This method convert the lidar pose to world frame

        Parameters
        ----------
        scene_number : int
            The scene number (see nuscenes basic tutorial)	
		sample_number : int
            The sample number (see nuscenes basic tutorial)
        Returns
        -------
        x,y : each is np.ndarray 
            lidar pose  x,y  in the world frame. notice we got rid of the z component
        """
        sensor='LIDAR_TOP'
        ego_pose = self.get_ego_pose(scene_number,sample_number)
        q = ego_pose['rotation']
        t = ego_pose['translation']
        T_ego_to_world = self.getT(q,t)
        token=self.get_sample_token(scene_number,sample_number)
        my_sample= nusc.get('sample', token)
        sensor_data= nusc.get('sample_data', my_sample['data'][sensor])
        qua=nusc.get('calibrated_sensor', sensor_data['calibrated_sensor_token'])['rotation']
        trans =nusc.get('calibrated_sensor', sensor_data['calibrated_sensor_token'])['translation']
        #trans= [0.943713, 0.0, 1.84023]
        #qua=[0.7077955119163518,-0.006492242056004365,0.010646214713995808,-0.7063073142877817]
        T_lidar_to_ego = self.getT(qua,trans)
        a=[0,0,0,1]  #zero of lidar axis with the 1 homogeneous
        lidar_pose=T_ego_to_world.dot(T_lidar_to_ego).dot(a)
        lidar_pose=np.delete(lidar_pose,2,0)
        lidar_pose=np.delete(lidar_pose,2,0)
        lidar_pose=lidar_pose.T
        return lidar_pose
#####################################################################
    def get_all_points(self,scene_number):
        """
        This method convert the lidar points clouds from a whole scene to world frame in addition to convert the coressponding lidar poses in the world frame as well

        Parameters
        ----------
        scene_number : int
            The scene number (see nuscenes basic tutorial)	
        Returns
        -------
       all_points : np.ndarray (2,n)
            lidar x,y points of the whole scene in the world frame. 
	   all_poses : np.ndarray (2,n)
            lidar x,y coressponding poses in the world frame.
        """
        samples_num=nusc.scene[scene_number]['nbr_samples']
        all_points=[]
        all_poses=[]
        for i in range(samples_num):
            sample=self.get_lidar_sample(scene_number,i)
            sample=np.array(sample,dtype=float).T
            sample_size=sample.shape[0]
            pose=self.get_lidar_pose(scene_number,i)
            for j in range(sample_size): 
                #pose=self.get_lidar_pose(scene_number,i)
                all_poses.append(pose)
                all_points.append(sample[j,:])            
        all_points=all_points
        all_points=np.array(all_points,dtype=float)
        return all_points,all_poses
 ###################################################       
    def get_grid_size_and_offset(self,lidar_points,lidar_poses,resolution):
        """
        This method calculate the size of the occupancy grid based on the minimum and maximum point in the data

        Parameters
        ----------
        lidar_points : np.ndarray
            lidar point cloud in the world fame
         lidar_poses : np.ndarray   
            lidar poses  in the world fame
        Returns
        -------
       x_size, y_size : int
            the size of the grid.
        x_offset,y_offset: float
             the minimum x and minimum y in among all the points
        """
        x_points_max=np.max(lidar_points[:,0])
        x_points_min=np.min(lidar_points[:,0])
        y_points_max=np.max(lidar_points[:,1])
        y_points_min=np.min(lidar_points[:,1])
        x_radar_max=np.max(lidar_poses[:,0])
        x_radar_min=np.min(lidar_poses[:,0])
        y_radar_max=np.max(lidar_poses[:,1])
        y_radar_min=np.min(lidar_poses[:,1])

        x_max=max(x_points_max,x_radar_max)
        y_max=max(y_points_max,y_radar_max)
        x_min=min(x_points_min,x_radar_min)
        y_min=min(y_points_min,y_radar_min)
        x_size=int(x_max-x_min)
        y_size=int(y_max-y_min)
        x_offset=x_min
        y_offset=y_min
        x_size =int((x_size+5)/resolution)   # 5 cells is a margain , you can change it 
        y_size =int((y_size +5)/resolution)
        return x_size, y_size, x_offset,y_offset
