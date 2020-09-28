import os
import signal
from subprocess import Popen
import time

class RosCore:

    def __init__(self):
        path_logs = '/home/tello18/.ros/log'
        path_core = '/opt/ros/melodic/bin/roscore'

        # Start the ROS Core
        p_ros_core = start_core([path_core], 'ros', dpath_logs)
        # print pids in case something goes wrong
        print('PGID ROS: ', os.getpgid(p_ros_core.pid))

    def run(self, cmd, stdout, stderr):
        """Run a given `cmd` in a subprocess, write logs to stdout / stderr.

        Parameters
        ----------
        cmd : list of str
            Command to run.
        stdout : str or subprocess.PIPE object
            Destination of stdout output.
        stderr : str or subprocess.PIPE object
            Destination of stderr output.

        Returns
        -------
        A subprocess.Popen instance.
        """
        return Popen(cmd, stdout=stdout, stderr=stderr, shell=False,
                     preexec_fn=os.setsid)


    def get_stdout_stderr(self, typ, datetime, dir):
        """Create stdout / stderr file paths."""
        out = '%s_%s_stdout.log' % (datetime, typ)
        err = '%s_%s_stderr.log' % (datetime, typ)
        return os.path.join(dir, out), os.path.join(dir, err)


    def start_core(self, cmd, typ, dpath_logs):
        """Start a subprocess 
        A subprocess.Popen instance.
        """
        start_time = time.strftime('%Y%m%d_%H%M%S')
        print('Starting', typ.upper())
        stdout, stderr = get_stdout_stderr(typ, start_time, dpath_logs)
        with open(stdout, 'wb') as out, open(stderr, 'wb') as err:
            return run(cmd, stdout=out, stderr=err)


    def kill_core(self):
        print('Killing ROS core')
        os.killpg(os.getpgid(p_ros_core.pid), signal.SIGTERM)

