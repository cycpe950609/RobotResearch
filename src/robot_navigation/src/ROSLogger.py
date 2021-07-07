from numpy.core.fromnumeric import var
import rosbag
from datetime import datetime
import robot_navigation.msg
import rospy
import std_msgs
from std_msgs.msg import Bool,String,Int64,Float64,UInt64
import threading

class ROSLogger(object):

    logger_cv = threading.Condition()

    def __init__(self,ifSaveLog,ifSaveTimeline):
        super(ROSLogger,self).__init__()
        self._ifSaveLog = ifSaveLog
        self._ifSaveTimeline = ifSaveTimeline

        # Get filename 
        date = datetime.now().strftime("%Y-%m-%d-%I-%M%p")
        name = date + "-roslogger.bag"
        self._bag = rosbag.Bag(name,'w')

    # def __del__(self):
    #     self._bag.close()

    def close(self):
        self._bag.close()
        self._ifSaveLog = False

    def _createROSLogMSG(self):
        roslog = robot_navigation.msg.ROSLog()
        roslog.header.frame_id = 'odom'
        roslog.header.stamp = rospy.Time.now()
        roslog.Type = 'NULL'
        roslog.About = ''
        roslog.Value = 'NULL'
        return roslog
    def _log(self,name,type_name,about,variable):
        if(self._ifSaveLog):
            # Save to /Name
            # if( not callable(variable) ): # if callable return True , it mean variable is a function
            var_name = '/' + name.replace(' ','_')
            self._bag.write( var_name ,variable)
            # Save to /timeline
            if(self._ifSaveTimeline):
                logv = self._createROSLogMSG()
                logv.Name = name
                logv.About = about
                logv.Type = type_name
                logv.Value = str(variable)
                
                self._bag.write('/timeline',logv)

    def LogFunction(self,FunctionName):
        # name,type_name,about,variable
        ss = String()
        ss.data = FunctionName
        self.logger_cv.acquire()# Do we need mutex ? If we dont have this, will two message store to bag simultaneously ?
        self._log(FunctionName,'Function','Function called',ss)
        self.logger_cv.notify()
        self.logger_cv.release()
    
    def LogBool(self,Name,About,boolean_value):
        # name,type_name,about,variable
        assert (type(boolean_value) is bool), 'Must be boolean'
        bl = Bool()
        bl.data = boolean_value
        self.logger_cv.acquire()# Do we need mutex ?
        self._log(Name,'Boolean',About,bl)
        self.logger_cv.notify()
        self.logger_cv.release()
    def LogNumber(self,Name,About,number):
        # name,type_name,about,variable
        assert (type(number) is int or type(number) is float), 'Must be Number'
        num = 0
        if(type(number) is int):
            num = Int64()
        elif(type(number) is float):
            num = Float64()
        num.data = number

        self.logger_cv.acquire()# Do we need mutex ?
        self._log(Name,'Number',About,num)
        self.logger_cv.notify()
        self.logger_cv.release()

    def LogString(self,Name,About,string_value):
        # name,type_name,about,variable
        assert (type(string_value) is str ), 'Must be String'
        ss = String()
        ss.data = string_value
        
        self.logger_cv.acquire()# Do we need mutex ?
        self._log(Name,'string',About,ss)
        self.logger_cv.notify()
        self.logger_cv.release()

    def Log(self, Name ,About ,variable ):
        # name,type_name,about,variable
        # assert hasattr(variable,_type), 'Must be Message type of ROS'
        self.logger_cv.acquire()# Do we need mutex ?
        self._log(Name,str(type(variable)),About,variable)
        self.logger_cv.notify()
        self.logger_cv.release()

