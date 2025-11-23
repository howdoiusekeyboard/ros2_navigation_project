import ROSLIB from 'roslib';

export interface TwistCommand {
  linear: number;
  angular: number;
}

export interface NavigateToPoseGoal {
  x: number;
  y: number;
  theta: number;
}

export interface RobotPose {
  x: number;
  y: number;
  theta: number;
}

export interface Explanation {
  timestamp: number;
  decision_type: string;
  text: string;
  confidence: number;
  metrics: {
    generation_time: number;
    token_count: number;
  };
}

class ROSService {
  private ros: ROSLIB.Ros | null = null;
  private cmdVelPublisher: ROSLIB.Topic | null = null;
  private poseSubscriber: ROSLIB.Topic | null = null;
  private connectionStatus: 'disconnected' | 'connecting' | 'connected' | 'error' = 'disconnected';
  private statusCallbacks: ((status: string) => void)[] = [];
  private poseCallbacks: ((pose: RobotPose) => void)[] = [];

  constructor() {
    this.connect();
  }

  /**
   * Connect to rosbridge websocket server
   */
  connect(url: string = 'ws://localhost:9090'): void {
    this.connectionStatus = 'connecting';
    this.notifyStatusChange('Connecting to ROS2...');

    this.ros = new ROSLIB.Ros({
      url: url
    });

    this.ros.on('connection', () => {
      console.log('Connected to rosbridge server');
      this.connectionStatus = 'connected';
      this.notifyStatusChange('Connected to ROS2');
      this.setupPublishers();
      this.setupSubscribers();
    });

    this.ros.on('error', (error) => {
      console.error('Error connecting to rosbridge:', error);
      this.connectionStatus = 'error';
      this.notifyStatusChange('ROS2 connection error');
    });

    this.ros.on('close', () => {
      console.log('Connection to rosbridge closed');
      this.connectionStatus = 'disconnected';
      this.notifyStatusChange('Disconnected from ROS2');
    });
  }

  /**
   * Setup publishers for robot control
   */
  private setupPublishers(): void {
    if (!this.ros) return;

    // Publisher for turtlesim (demo) - topic: /turtle1/cmd_vel
    this.cmdVelPublisher = new ROSLIB.Topic({
      ros: this.ros,
      name: '/turtle1/cmd_vel',
      messageType: 'geometry_msgs/Twist'
    });
  }

  /**
   * Setup subscribers for robot state
   */
  private explanationSubscriber: ROSLIB.Topic | null = null;
  private explanationCallbacks: ((explanation: Explanation) => void)[] = [];

  /**
   * Setup subscribers for robot state
   */
  private setupSubscribers(): void {
    if (!this.ros) return;

    // Subscribe to robot pose (if available)
    // For turtlesim, this would be /turtle1/pose
    this.poseSubscriber = new ROSLIB.Topic({
      ros: this.ros,
      name: '/turtle1/pose',
      messageType: 'turtlesim/Pose'
    });

    this.poseSubscriber.subscribe((message: any) => {
      const pose: RobotPose = {
        x: message.x,
        y: message.y,
        theta: message.theta
      };
      this.notifyPoseChange(pose);
    });

    // Subscribe to explanations
    this.explanationSubscriber = new ROSLIB.Topic({
      ros: this.ros,
      name: '/navigation/explanation_detailed',
      messageType: 'std_msgs/String'
    });

    this.explanationSubscriber.subscribe((message: any) => {
      try {
        const explanation: Explanation = JSON.parse(message.data);
        this.notifyExplanationReceived(explanation);
      } catch (e) {
        console.error('Failed to parse explanation:', e);
      }
    });
  }

  /**
   * Register callback for explanation updates
   */
  onExplanationReceived(callback: (explanation: Explanation) => void): void {
    this.explanationCallbacks.push(callback);
  }

  /**
   * Notify all explanation callbacks
   */
  private notifyExplanationReceived(explanation: Explanation): void {
    this.explanationCallbacks.forEach(cb => cb(explanation));
  }

  /**
   * Publish Twist command to robot
   */
  publishTwist(command: TwistCommand): void {
    if (!this.cmdVelPublisher || this.connectionStatus !== 'connected') {
      console.error('Cannot publish: ROS not connected');
      return;
    }

    const twist = new ROSLIB.Message({
      linear: {
        x: command.linear,
        y: 0,
        z: 0
      },
      angular: {
        x: 0,
        y: 0,
        z: command.angular
      }
    });

    this.cmdVelPublisher.publish(twist);
    console.log('Published twist:', command);
  }

  /**
   * Stop the robot immediately
   */
  emergencyStop(): void {
    this.publishTwist({ linear: 0, angular: 0 });
  }

  /**
   * Get current connection status
   */
  getConnectionStatus(): string {
    return this.connectionStatus;
  }

  /**
   * Register callback for connection status changes
   */
  onStatusChange(callback: (status: string) => void): void {
    this.statusCallbacks.push(callback);
  }

  /**
   * Register callback for pose updates
   */
  onPoseChange(callback: (pose: RobotPose) => void): void {
    this.poseCallbacks.push(callback);
  }

  /**
   * Notify all status change callbacks
   */
  private notifyStatusChange(status: string): void {
    this.statusCallbacks.forEach(cb => cb(status));
  }

  /**
   * Notify all pose change callbacks
   */
  private notifyPoseChange(pose: RobotPose): void {
    this.poseCallbacks.forEach(cb => cb(pose));
  }

  /**
   * Disconnect from ROS
   */
  disconnect(): void {
    if (this.ros) {
      this.ros.close();
    }
  }
}

// Singleton instance
export const rosService = new ROSService();
