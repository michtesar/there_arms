import threading

from kortex_api.RouterClient import RouterClient, RouterClientSendOptions
from kortex_api.SessionManager import SessionManager
from kortex_api.TCPTransport import TCPTransport
from kortex_api.UDPTransport import UDPTransport
from kortex_api.autogen.messages import Base_pb2
from kortex_api.autogen.messages import Session_pb2


class Pose:

    def __init__(self, pose_array):
        self.x = pose_array[0]  # m
        self.y = pose_array[1]  # m
        self.z = pose_array[2]  # m
        self.theta_x = pose_array[3]  # deg
        self.theta_y = pose_array[4]  # deg
        self.theta_z = pose_array[5]  # deg

    def __str__(self):
        return f"Position:\n\tx: {self.x}\n\ty: {self.y}\n\tz: {self.z}\nRotation:\n\tx: " \
               f"{self.theta_x}\n\ty: {self.theta_y}\n\tz: {self.theta_z}"


class DeviceConnection:
    TCP_PORT = 10000
    UDP_PORT = 10001

    @staticmethod
    def createTcpConnection(args):
        """
        returns RouterClient required to create services and send requests to device or sub-devices,
        """

        return DeviceConnection(args.ip, port=DeviceConnection.TCP_PORT, credentials=(args.username, args.password))

    @staticmethod
    def createUdpConnection(args):
        """
        returns RouterClient that allows to create services and send requests to a device or its sub-devices @ 1khz.
        """

        return DeviceConnection(args.ip, port=DeviceConnection.UDP_PORT, credentials=(args.username, args.password))

    def __init__(self, ipAddress, port=TCP_PORT, credentials=("", "")):

        self.ipAddress = ipAddress
        self.port = port
        self.credentials = credentials

        self.sessionManager = None

        # Setup API
        self.transport = TCPTransport() if port == DeviceConnection.TCP_PORT else UDPTransport()
        self.router = RouterClient(self.transport, RouterClient.basicErrorCallback)

    # Called when entering 'with' statement
    def __enter__(self):

        self.transport.connect(self.ipAddress, self.port)

        if (self.credentials[0] != ""):
            session_info = Session_pb2.CreateSessionInfo()
            session_info.username = self.credentials[0]
            session_info.password = self.credentials[1]
            session_info.session_inactivity_timeout = 10000  # (milliseconds)
            session_info.connection_inactivity_timeout = 2000  # (milliseconds)

            self.sessionManager = SessionManager(self.router)
            print("Logging as", self.credentials[0], "on device", self.ipAddress)
            self.sessionManager.CreateSession(session_info)

        return self.router

    # Called when exiting 'with' statement
    def __exit__(self, exc_type, exc_value, traceback):

        if self.sessionManager != None:
            router_options = RouterClientSendOptions()
            router_options.timeout_ms = 1000

            self.sessionManager.CloseSession(router_options)

        self.transport.disconnect()


class Robot:
    TIMEOUT_DURATION = 20
    POSE_RETRACT = "Retract"
    POSE_HOME = "Home"
    POSE_ZERO = "Zero"
    POSE_PACKAGING = "Packaging"

    def __init__(self, ip_address):
        self.ip_address = ip_address

    @staticmethod
    def check_for_sequence_end_or_abort(e):
        def check(notification, e=e):
            event_id = notification.event_identifier
            task_id = notification.task_index
            if event_id == Base_pb2.SEQUENCE_TASK_COMPLETED:
                print("Sequence task {} completed".format(task_id))
            elif event_id == Base_pb2.SEQUENCE_ABORTED:
                print("Sequence aborted with error {}:{}".format(
                    notification.abort_details,
                    Base_pb2.SubErrorCodes.Name(notification.abort_details))
                )
                e.set()
            elif event_id == Base_pb2.SEQUENCE_COMPLETED:
                print("Sequence completed.")
                e.set()

        return check

    @staticmethod
    def check_for_end_or_abort(e):
        """Return a closure checking for END or ABORT notifications
        Arguments:
        e -- event to signal when the action is completed
            (will be set when an END or ABORT occurs)
        """

        def check(notification, e=e):
            print("EVENT : " + \
                  Base_pb2.ActionEvent.Name(notification.action_event))
            if notification.action_event == Base_pb2.ACTION_END \
                    or notification.action_event == Base_pb2.ACTION_ABORT:
                e.set()

        return check

    def move_named_pose(self, base, name="Retract"):
        base_servo_mode = Base_pb2.ServoingModeInformation()
        base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
        base.SetServoingMode(base_servo_mode)

        # Move arm to ready position
        print("Moving the arm to a safe position")
        action_type = Base_pb2.RequestedActionType()
        action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
        action_list = base.ReadAllActions(action_type)
        action_handle = None
        for action in action_list.action_list:
            if action.name == name:
                action_handle = action.handle

        if action_handle is None:
            print("Can't reach safe position. Exiting")
            return False

        e = threading.Event()
        notification_handle = base.OnNotificationActionTopic(
            self.check_for_end_or_abort(e),
            Base_pb2.NotificationOptions()
        )

        base.ExecuteActionFromReference(action_handle)
        finished = e.wait(self.TIMEOUT_DURATION)
        base.Unsubscribe(notification_handle)

        if finished:
            print("Safe position reached")
        else:
            print("Timeout on action notification wait")
        return finished

    def move_angular(self, base, configuration):
        assert len(configuration) == 6

        action = Base_pb2.Action()
        action.name = "[move_angular] Motion"
        action.application_data = ""
        actuator_count = base.GetActuatorCount()

        # Place arm straight up
        for joint_id, join_value in zip(range(actuator_count.count), configuration):
            joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
            joint_angle.joint_identifier = joint_id
            joint_angle.value = join_value

        e = threading.Event()
        notification_handle = base.OnNotificationActionTopic(
            self.check_for_end_or_abort(e),
            Base_pb2.NotificationOptions()
        )

        print("Executing action")
        base.ExecuteAction(action)

        print("Waiting for movement to finish ...")
        finished = e.wait(self.TIMEOUT_DURATION)
        base.Unsubscribe(notification_handle)

        if finished:
            print("Angular movement completed")
        else:
            print("Timeout on action notification wait")
        return finished

    @staticmethod
    def get_pose(base_cyclic):
        action = Base_pb2.Action()
        action.name = "Get cartesian pose"
        action.application_data = ""

        feedback = base_cyclic.RefreshFeedback()
        data = feedback.base
        return Pose([data.tool_pose_x, data.tool_pose_y, data.tool_pose_z, data.tool_pose_theta_x,
                     data.tool_pose_theta_y, data.tool_pose_theta_z])

    @staticmethod
    def get_joints(base_cyclic, velocity=False):
        action = Base_pb2.Action()
        action.name = "Get cartesian pose"
        action.application_data = ""

        feedback = base_cyclic.RefreshFeedback()
        data = feedback.actuators
        if velocity:
            return [[i.position for i in data], [i.velocity for i in data]]
        else:
            return [i.position for i in data]

    @staticmethod
    def create_joint_sequence(base, joint_array):
        action = Base_pb2.Action()
        action.name = "Joint sequence"
        action.application_data = ""

        actuator_count = base.GetActuatorCount().count
        for joint_id, joint_value in zip(range(actuator_count), joint_array):
            joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
            joint_angle.value = joint_value

        return action

    @staticmethod
    def create_cartesian_absolute_sequence(pose):
        action = Base_pb2.Action()
        action.name = "Cartesian sequence"
        action.application_data = ""

        cartesian_pose = action.reach_pose.target_pose
        cartesian_pose.x = pose.x
        cartesian_pose.y = pose.y
        cartesian_pose.z = pose.z
        cartesian_pose.theta_x = pose.theta_x
        cartesian_pose.theta_y = pose.theta_y
        cartesian_pose.theta_z = pose.theta_z

        return action

    @staticmethod
    def create_cartesian_action(base_cyclic, pose):
        action = Base_pb2.Action()
        action.name = "Cartesian action"
        action.application_data = ""

        cartesian_pose = action.reach_pose.target_pose
        feedback = base_cyclic.RefreshFeedback()
        if type(pose) == Pose:
            print("Creating absolute cartesian action")
            cartesian_pose.x = pose.x
            cartesian_pose.y = pose.y
            cartesian_pose.z = pose.z
            cartesian_pose.theta_x = pose.theta_x
            cartesian_pose.theta_y = pose.theta_y
            cartesian_pose.theta_z = pose.theta_z
        elif type(pose) == list:
            if len(pose) >= 3:
                print("Creating relative cartesian action")
                cartesian_pose.x = feedback.base.tool_pose_x + pose[0]
                cartesian_pose.y = feedback.base.tool_pose_y + pose[1]
                cartesian_pose.z = feedback.base.tool_pose_z + pose[2]
                cartesian_pose.theta_x = feedback.base.tool_pose_theta_x
                cartesian_pose.theta_y = feedback.base.tool_pose_theta_y
                cartesian_pose.theta_z = feedback.base.tool_pose_theta_z
                if len(pose) == 6:
                    cartesian_pose.theta_x += pose[3]
                    cartesian_pose.theta_y += pose[4]
                    cartesian_pose.theta_z += pose[5]
        return action

    def move_sequence(self, base, task_list):
        print(f"Creating a sequence of {len(task_list)} task(s)")
        sequence = Base_pb2.Sequence()
        sequence.name = f"Sequence of {len(task_list)} tasks"
        for i, task in enumerate(task_list):
            current_tasks = sequence.tasks.add()
            current_tasks.group_identifier = i
            current_tasks.action.CopyFrom(task)

        print("Sequence was prepared")
        e = threading.Event()
        notification_handle = base.OnNotificationSequenceInfoTopic(
            self.check_for_sequence_end_or_abort(e),
            Base_pb2.NotificationOptions()
        )

        print("Creating sequence on device and executing it")
        handle_sequence = base.CreateSequence(sequence)
        base.PlaySequence(handle_sequence)

        print("Waiting for movement to finish ...")
        finished = e.wait(self.TIMEOUT_DURATION)
        base.Unsubscribe(notification_handle)

        if not finished:
            print("Timeout on action notification wait")
        return finished


class KinovaGen3Lite(Robot):

    def __init__(self, ip_address='192.178.1.10'):
        super().__init__(ip_address)
