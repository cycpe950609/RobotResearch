import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QGraphicsScene
from python_qt_binding.QtCore import  Qt

from .interactive_graphics_view import InteractiveGraphicsView

from nav_msgs.msg import Odometry

class MapGraph(Plugin):

    def getPoseCallback(self,msg):
        # print ( "Position %f %f %f \n" )%(msg.pose.pose.position.x , msg.pose.pose.position.y , msg.pose.pose.position.z)
        pass

    # Mouse Event
    def mousePressEvent(self,mouseEvent):
        # print "TEST"
        print("mousePressEvent",mouseEvent.type(),mouseEvent.pos())
        
        
    # Map Draw

    def _draw_coordinate(self):
        pass
    def _draw_start_point(self):
        pass
    def _draw_destination_point(self):
        pass
    def _draw_turtlebot(self):
        pass
    def _draw_laser_distance(self):
        pass

    def _redraw_map(self):
        if(self._widget.coordinate_check_box.isChecked()):
            self._draw_coordinate()
        if(self._widget.start_point_check_box.isChecked()):
            self._draw_start_point()
        if(self._widget.destitution_check_box.isChecked()):
            self._draw_destination_point()
        if(self._widget.turtlebot_check_box.isChecked()):
            self._draw_turtlebot()
        if(self._widget.laser_disance_check_box.isChecked()):
            self._draw_laser_distance()

    def __init__(self, context):
        super(MapGraph, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MapGraph')
        self.initialized = False


        # # Process standalone plugin command-line arguments
        # from argparse import ArgumentParser
        # parser = ArgumentParser()
        # # Add argument(s) to the parser.
        # parser.add_argument("-q", "--quiet", action="store_true",
        #               dest="quiet",
        #               help="Put plugin in silent mode")
        # args, unknowns = parser.parse_known_args(context.argv())
        # if not args.quiet:
        #     print 'arguments: ', args
        #     print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_destination_clicker'), 'resource','MapGraph.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget,{'InteractiveGraphicsView': InteractiveGraphicsView})
        # Give QObjects reasonable names
        self._widget.setObjectName('MapGraphUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # CheckBox change Value
        self._widget.start_point_check_box.stateChanged.connect(self._update_graph)
        self._widget.destitution_check_box.stateChanged.connect(self._update_graph)
        self._widget.coordinate_check_box.stateChanged.connect(self._update_graph)

        # MapGraph
        self._scene = QGraphicsScene()
        self._scene.setBackgroundBrush(Qt.white)
        self._widget.graphics_view.setScene(self._scene)
        # self._widget.graphics_view.mousePressEvent = self.mousePressEvent
        self._redraw_map()

        # Init Node
        # rospy.init_node('destination_clicker')
        sub = rospy.Subscriber('/odom',Odometry,self.getPoseCallback)

    def _update_graph(self):
        if not self.initialized:
            return
        self._redraw_map()

   

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        instance_settings.set_value('start_point_check_box_state',self._widget.start_point_check_box.isChecked() )
        instance_settings.set_value('destitution_check_box_state',self._widget.destitution_check_box.isChecked() )
        instance_settings.set_value('coordinate_check_box_state',self._widget.coordinate_check_box.isChecked() )
        instance_settings.set_value('turtlebot_check_box_state',self._widget.turtlebot_check_box.isChecked() )
        instance_settings.set_value('laser_disance_check_box_state',self._widget.laser_disance_check_box.isChecked() )

        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)

        self._widget.start_point_check_box.setChecked(instance_settings.value('start_point_check_box_state', True) in [True, 'true'])
        self._widget.destitution_check_box.setChecked(instance_settings.value('destitution_check_box_state', True) in [True, 'true'])
        self._widget.coordinate_check_box.setChecked(instance_settings.value('coordinate_check_box_state', True) in [True, 'true'])
        self._widget.turtlebot_check_box.setChecked(instance_settings.value('turtlebot_check_box_state', True) in [True, 'true'])
        self._widget.laser_disance_check_box.setChecked(instance_settings.value('laser_disance_check_box_state', True) in [True, 'true'])

        self.initialized = True
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
