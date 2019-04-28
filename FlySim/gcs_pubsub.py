import sys
import time
import threading
import zmq
from PyQt4 import QtCore, QtGui, uic

qtCreatorFile = "GUI/mainwindow.ui"
Ui_MainWindow, QtBaseClass = uic.loadUiType(qtCreatorFile)


class MyGCS(QtGui.QMainWindow, Ui_MainWindow):
    def __init__(self, tel_port, control_port, uav_instances, verbose):
        self.uav_count = uav_instances
        self.tel_msg_count = 0
        self.control_msg_count = 0
        self.verbose = verbose
        self.running = True
        self.prefix = "[GCS]"

        # Setting up ZMQ related parameters
        self.zmq_tel_port = tel_port
        self.zmq_control_port = control_port
        self.zmq_tel_connection_str = "tcp://127.0.0.1:" + str(self.zmq_tel_port)
        self.zmq_control_connection_str = "tcp://127.0.0.1:" + str(self.zmq_control_port)
        self.zmq_tel_socket = self.create_zmq("SUB", self.zmq_tel_connection_str, "", verbose=self.verbose)
        self.zmq_control_socket = self.create_zmq("PUB", self.zmq_control_connection_str, verbose=self.verbose)

        QtGui.QMainWindow.__init__(self)
        Ui_MainWindow.__init__(self)
        self.setupUi(self)
        self.lbl_status.setText("Disconnected")
        self.btn_connect.clicked.connect(lambda: self.connect_uav())
        self.btn_go.clicked.connect(lambda: self.go_to())
        self.btn_rtl.clicked.connect(lambda: self.rtl())
        self.btn_up.clicked.connect(lambda: self.go_up())
        self.btn_down.clicked.connect(lambda: self.go_down())
        self.btn_forward.clicked.connect(lambda: self.go_forward())
        self.btn_backward.clicked.connect(lambda: self.go_backward())
        self.btn_left.clicked.connect(lambda: self.go_left())
        self.btn_right.clicked.connect(lambda: self.go_right())
        self.btn_takeoff.clicked.connect(lambda: self.takeoff())
        self.btn_land.clicked.connect(lambda: self.land())
        self.btn_arm_disarm.clicked.connect(lambda: self.arm_disarm_throttle())
        self.cb_mode.addItem("GUIDED")
        self.cb_mode.addItem("LOITER")
        self.cb_mode.setCurrentIndex(0)
        self.btn_arm_disarm.setEnabled(False)
        self.btn_go.setEnabled(False)
        self.btn_up.setEnabled(False)
        self.btn_down.setEnabled(False)
        self.btn_left.setEnabled(False)
        self.btn_right.setEnabled(False)
        self.btn_forward.setEnabled(False)
        self.btn_backward.setEnabled(False)
        self.btn_takeoff.setEnabled(False)
        self.btn_land.setEnabled(False)
        self.btn_rtl.setEnabled(False)
        self.flying = 0
        self.arm = 0

        # Thread for Telemetry
        self.thread_tel = threading.Thread(target=self.get_telemetry)
        self.thread_tel.setName("GCS-telemetry")
        self.thread_tel.daemon = True
        self.thread_tel.start()

        # Thread for heartbeat
        self.thread_heartbeat = threading.Thread(target=self.generate_heartbeat)
        self.thread_heartbeat.setName("GCS-heartbeat")
        self.thread_heartbeat.daemon = True
        self.thread_heartbeat.start()

    def create_zmq(self, zmq_type, con_string, prefix="", verbose=False):
        context = zmq.Context()
        if "PUB" in zmq_type:
            if verbose:
                print(self.prefix + " [ZMQ] Binding publisher started " + con_string)
            sock_new = context.socket(zmq.PUB)
            sock_new.bind(con_string)
            if verbose:
                print(self.prefix + " [ZMQ] Publisher bound complete " + con_string)
        elif "SUB" in zmq_type:
            if verbose:
                print(self.prefix + " [ZMQ] Subscriber connect started " + con_string)
            sock_new = context.socket(zmq.SUB)
            sock_new.connect(con_string)
            sock_new.setsockopt(zmq.SUBSCRIBE, prefix)
            if verbose:
                print(self.prefix + " [ZMQ] Subscriber connect complete " + con_string)
        else:
            return None
        return sock_new

    def connection_close(self):
        if self.verbose:
            print(self.prefix + " Closing the connections")
        print(self.prefix + " From desctructor")
        self.send_data("TERMINATE", "000", self.zmq_control_socket, self.verbose)
        self.running = False
        self.zmq_control_socket.close()
        self.zmq_tel_socket.close()

    def connect_uav(self):
        for i in range(self.uav_count):
            self.send_data("COMMAND:CONNECT", format(i, "03d"), self.zmq_control_socket, self.verbose)

    def arm_disarm_throttle(self):
        if not self.arm:
            cmd = "COMMAND:ARM"
            for i in range(self.uav_count):
                self.send_data(cmd, format(i, "03d"), self.zmq_control_socket, self.verbose)
            self.arm = 1
            self.lbl_status.setText("Arm Requested")
        elif self.arm:
            cmd = "COMMAND:DISARM"
            for i in range(self.uav_count):
                self.send_data(cmd, format(i, "03d"), self.zmq_control_socket, self.verbose)
            self.arm = 0
            self.lbl_status.setText("Disarm Requested")

    def takeoff(self):
        if self.arm:
            self.lbl_status.setText("Takeoff Requested")
            alt = self.sb_alt.value()
            speed = self.sb_speed.value()
            mode = self.cb_mode.currentText()
            cmd = "COMMAND:TAKEOFF|ALT=" + str(alt) + "|MODE=" + str(mode) + "|SPEED=" + str(speed)
            for i in range(self.uav_count):
                self.send_data(cmd, format(i, "03d"), self.zmq_control_socket, self.verbose)
            self.flying = 1

    def land(self):
        if self.flying:
            self.lbl_status.setText("Land Requested")
            cmd = "COMMAND:LAND"
            for i in range(self.uav_count):
                self.send_data(cmd, format(i, "03d"), self.zmq_control_socket, self.verbose)
            self.send_data(cmd, "000", self.zmq_control_socket, self.verbose)
            self.flying = 0

    def rtl(self):
        if self.flying:
            self.lbl_status.setText("RTL Requested")
            cmd = "COMMAND:RTL"
            for i in range(self.uav_count):
                self.send_data(cmd, format(i, "03d"), self.zmq_control_socket, self.verbose)

    def go_to(self):
        if self.flying:
            x = self.tb_goto_x.text()
            y = self.tb_goto_y.text()
            self.lbl_status.setText("Goto: X=" + str(x) + " Y=" +str(y))
            cmd = "COMMAND:GOTO|X=" + str(x) + "|Y=" + str(y)
            print(cmd)
            self.send_data(cmd, "000", self.zmq_control_socket, self.verbose)

    def go_up(self):
        if self.flying:
            self.lbl_status.setText("Up")
            cmd = "COMMAND:GO_UP"
            for i in range(self.uav_count):
                self.send_data(cmd, format(i, "03d"), self.zmq_control_socket, self.verbose)

    def go_down(self):
        if self.flying:
            self.lbl_status.setText("Down")
            cmd = "COMMAND:GO_DOWN"
            for i in range(self.uav_count):
                self.send_data(cmd, format(i, "03d"), self.zmq_control_socket, self.verbose)

    def go_left(self):
        if self.flying:
            self.lbl_status.setText("Left")
            cmd = "COMMAND:GO_LEFT"
            for i in range(self.uav_count):
                self.send_data(cmd, format(i, "03d"), self.zmq_control_socket, self.verbose)

    def go_right(self):
        if self.flying:
            self.lbl_status.setText("Right")
            cmd = "COMMAND:GO_RIGHT"
            for i in range(self.uav_count):
                self.send_data(cmd, format(i, "03d"), self.zmq_control_socket, self.verbose)

    def go_forward(self):
        if self.flying:
            self.lbl_status.setText("Forward")
            cmd = "COMMAND:GO_FORWARD"
            for i in range(self.uav_count):
                self.send_data(cmd, format(i, "03d"), self.zmq_control_socket, self.verbose)

    def go_backward(self):
        if self.flying:
            self.lbl_status.setText("Backward")
            cmd = "COMMAND:GO_BACKWARD"
            for i in range(self.uav_count):
                self.send_data(cmd, format(i, "03d"), self.zmq_control_socket, self.verbose)

    def send_data(self, message, uav_id, sock, verbose):
        try:
            self.control_msg_count += 1
            msg_send = "@@@G_" + uav_id + "***" + str(self.control_msg_count) + "***" + \
                       repr(time.time()) + "***" + message + "***"
                       #str(time.time()) + "***" + message + "***"
            if verbose:
                print(self.prefix + " CONTROL: sending " + msg_send)
            self.textedit_control.append(msg_send + "\n")
            sock.send(msg_send)
        except:
            print(self.prefix + " CONTROL: Exception occurred while sending data")

    def get_telemetry(self):
        while self.running:
            data = self.zmq_tel_socket.recv()
            if data:
                self.textedit_telemetry.append(data + "\n")
                d_list = data.split('***')
                payload = d_list[3]
                if "TELEMETRY" in payload:
                    telemetry_data = d_list[3].split('#')
                    for t in telemetry_data:
                        if "LocationGlobalRelative" in t:
                            self.lbl_gps.setText(t[t.find(':')+1:])
                        elif "Battery" in t:
                            self.lbl_battery.setText(t[t.find(':')+1:])
                if "SENSOR" in payload:
                    print(data)
                if "STATUS" in payload:
                    if "DISCONNECT|True" in payload:
                        self.lbl_status.setText("Disconnected")
                        self.btn_connect.setText("Connect")
                        self.btn_arm_disarm.setEnabled(False)
                        self.btn_takeoff.setEnabled(False)
                        self.btn_land.setEnabled(False)
                        self.btn_rtl.setEnabled(False)
                        self.btn_go.setEnabled(False)
                        self.btn_up.setEnabled(False)
                        self.btn_down.setEnabled(False)
                        self.btn_left.setEnabled(False)
                        self.btn_right.setEnabled(False)
                        self.btn_forward.setEnabled(False)
                        self.btn_backward.setEnabled(False)
                    elif "CONNECT|True" in payload:
                        self.lbl_status.setText("Connected")
                        self.btn_connect.setText("Disconnect")
                        self.btn_arm_disarm.setEnabled(True)
                        self.btn_takeoff.setEnabled(False)
                        self.btn_land.setEnabled(False)
                        self.btn_rtl.setEnabled(False)
                        self.btn_go.setEnabled(False)
                        self.btn_up.setEnabled(False)
                        self.btn_down.setEnabled(False)
                        self.btn_left.setEnabled(False)
                        self.btn_right.setEnabled(False)
                        self.btn_forward.setEnabled(False)
                        self.btn_backward.setEnabled(False)
                    elif "DISARM|True" in payload:
                        self.lbl_status.setText("Disarm Complete")
                        self.btn_arm_disarm.setText("Arm")
                        self.btn_connect.setEnabled(True)
                        self.btn_takeoff.setEnabled(False)
                        self.btn_go.setEnabled(False)
                        self.btn_up.setEnabled(False)
                        self.btn_down.setEnabled(False)
                        self.btn_left.setEnabled(False)
                        self.btn_right.setEnabled(False)
                        self.btn_forward.setEnabled(False)
                        self.btn_backward.setEnabled(False)
                    elif "ARM|True" in payload:
                        self.lbl_status.setText("Arm Complete")
                        self.btn_takeoff.setEnabled(True)
                        self.btn_arm_disarm.setText("Disarm")
                        self.btn_connect.setEnabled(False)
                        self.btn_go.setEnabled(False)
                        self.btn_up.setEnabled(False)
                        self.btn_down.setEnabled(False)
                        self.btn_left.setEnabled(False)
                        self.btn_right.setEnabled(False)
                        self.btn_forward.setEnabled(False)
                        self.btn_backward.setEnabled(False)
                    elif "TAKEOFF|True" in payload:
                        self.lbl_status.setText("Takeoff Complete")
                        self.btn_land.setEnabled(True)
                        self.btn_rtl.setEnabled(True)
                        self.btn_takeoff.setEnabled(False)
                        self.btn_arm_disarm.setEnabled(False)
                        self.btn_connect.setEnabled(False)
                        self.btn_go.setEnabled(True)
                        self.btn_up.setEnabled(True)
                        self.btn_down.setEnabled(True)
                        self.btn_left.setEnabled(True)
                        self.btn_right.setEnabled(True)
                        self.btn_forward.setEnabled(True)
                        self.btn_backward.setEnabled(True)
                    elif "LAND|True" in payload:
                        self.lbl_status.setText("Land Complete")
                        self.btn_arm_disarm.setEnabled(True)
                        self.btn_arm_disarm.setText("Arm")
                        self.btn_connect.setEnabled(True)
                        self.btn_land.setEnabled(False)
                        self.btn_rtl.setEnabled(False)
                        self.btn_takeoff.setEnabled(False)
                        self.btn_go.setEnabled(False)
                        self.btn_up.setEnabled(False)
                        self.btn_down.setEnabled(False)
                        self.btn_left.setEnabled(False)
                        self.btn_right.setEnabled(False)
                        self.btn_forward.setEnabled(False)
                        self.btn_backward.setEnabled(False)
                    elif "RTL|True" in payload:
                        self.lbl_status.setText("RTL Complete")
                        self.btn_arm_disarm.setEnabled(True)
                        self.btn_arm_disarm.setText("Arm")
                        self.btn_connect.setEnabled(True)
                        self.btn_land.setEnabled(False)
                        self.btn_rtl.setEnabled(False)
                        self.btn_takeoff.setEnabled(False)
                        self.btn_go.setEnabled(False)
                        self.btn_up.setEnabled(False)
                        self.btn_down.setEnabled(False)
                        self.btn_left.setEnabled(False)
                        self.btn_right.setEnabled(False)
                        self.btn_forward.setEnabled(False)
                        self.btn_backward.setEnabled(False)
                    else:
                        self.lbl_status.setText(data)
                if self.verbose:
                    print(self.prefix + " TELEMETRY: Message received :" + data)

    def generate_heartbeat(self):
        while self.running:
            if self.arm:
                msg = "COMMAND:HEARTBEAT_MESSAGE"
                if self.verbose:
                    print(self.prefix + " CONTROL: HEARTBEAT sent")
                for i in range(self.uav_count):
                    self.send_data(msg, format(i, "03d"), self.zmq_control_socket, self.verbose)
                time.sleep(0.5)
            else:
                time.sleep(0.1)


def main(tel_port, control_port, instance, verbose):
    app = QtGui.QApplication(sys.argv)
    window = MyGCS(tel_port, control_port, instance, verbose)
    window.show()
    sys.exit(app.exec_())
    #app.exec_()
    window.connection_close()

