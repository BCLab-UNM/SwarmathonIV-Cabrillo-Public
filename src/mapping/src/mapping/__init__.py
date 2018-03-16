import rospy 

class RoverMap:
    '''RoverMap - A convenience class that wraps a GridMap message. 
    
    Attributes: 
    
        map : (GridMap) The original GridMap message 
        frame_id : (str) The frame ID that the GridMap is referenced to 
        stamp : (int) The last time the map was updated.
        resolution : (int) The spatial resolution of the map 
        length_x : (float) The size of the X axis in meters. 
        length_y : (float) The size of the Y axis in meters. 
        pose : (geometry_msgs.msg.Pose) The center of the map. 
        
    Accessing the map.
    
        GridMaps are built with layers. Layers are accessible by their layer
        name like a dictionary. For example, to access the "targets" layer:
        
            r_map['targets'] 
            
        Each layer is a numpy array. Check out the numpy tutorial here: 
        
            https://docs.scipy.org/doc/numpy-dev/user/quickstart.html
            
        The numpy funciton reference is here: 
        
            https://docs.scipy.org/doc/numpy/reference/routines.html
    '''    
    def __init__(self, m):
        self.map = m 
        self.frame_id = self.map.info.header.frame_id 
        self.stamp = self.map.info.header.stamp
        self.resolution = self.map.info.resolution
        self.length_x = self.map.info.length_x
        self.length_y = self.map.info.length_y 
        self.pose = self.map.info.pose        
        for n in self.map.data : 
            n.data.resize(n.layout.dim[0].size, n.layout.dim[1].size)
            n.data = n.data.T

    def __getitem__(self, item):
        if item not in self.map.layers :
            raise KeyError('The GridMap has no layer named ' + str(item))
        return self.map.data[self.map.layers.index(item)].data
    
    def __iter__(self):
        return iter(self.map.layers)
