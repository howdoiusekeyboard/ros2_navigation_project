import { Ros, Topic } from 'roslib';

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

export interface TwinSensorDiff {
  position_diff_x?: number;
  position_diff_y?: number;
  position_diff_total?: number;
  orientation_diff?: number;
  linear_vel_diff?: number;
  angular_vel_diff?: number;
  scan_diff_mean?: number;
  scan_diff_max?: number;
  scan_diff_variance?: number;
}

export interface AnomalyAlert {
  is_anomaly: boolean;
  score: number;
  threshold: number;
  severity: number;
  explanation: string;
  recommended_action: string;
  features?: TwinSensorDiff;
}

class ROSService {
  private ros: Ros | null = null;
  private cmdVelPublisher: Topic | null = null;
  private poseSubscriber: Topic | null = null;
  private connectionStatus: 'disconnected' | 'connecting' | 'connected' | 'error' = 'disconnected';
  private statusCallbacks: ((status: string) => void)[] = [];
  private poseCallbacks: ((pose: RobotPose) => void)[] = [];
  private rosbridgeUrl: string;

  constructor() {
    this.rosbridgeUrl = (import.meta as any).env?.VITE_ROSBRIDGE_URL || 'ws://localhost:9090';
    this.connect(this.rosbridgeUrl);
  }

  /**
   * Connect to rosbridge websocket server
   */
  connect(url: string = 'ws://localhost:9090'): void {
    this.rosbridgeUrl = url;
    this.connectionStatus = 'connecting';
    this.notifyStatusChange('Connecting to ROS2...');

    this.ros = new Ros({
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
    this.cmdVelPublisher = new Topic({
      ros: this.ros,
      name: '/turtle1/cmd_vel',
      messageType: 'geometry_msgs/Twist'
    });
  }

  /**
   * Setup subscribers for robot state
   */
  private explanationSubscriber: Topic | null = null;
  private explanationCallbacks: ((explanation: Explanation) => void)[] = [];
  private realOdomSubscriber: Topic | null = null;
  private twinSensorDiffSubscriber: Topic | null = null;
  private anomalyScoreSubscriber: Topic | null = null;
  private anomalyAlertSubscriber: Topic | null = null;

  private twinSensorDiffCallbacks: ((diff: TwinSensorDiff) => void)[] = [];
  private anomalyScoreCallbacks: ((score: number) => void)[] = [];
  private anomalyAlertCallbacks: ((alert: AnomalyAlert) => void)[] = [];
  private batterySubscriber: Topic | null = null;
  private batteryCallbacks: ((battery: { percentage: number; voltage: number }) => void)[] = [];

  /**
   * Setup subscribers for robot state
   */
  private setupSubscribers(): void {
    if (!this.ros) return;

    // Subscribe to turtlesim pose (demo mode)
    this.poseSubscriber = new Topic({
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

    // Subscribe to real robot odometry (if available) via /real/odom
    // This is populated when digital_twin_pkg's real_robot_bridge_node is running.
    this.realOdomSubscriber = new Topic({
      ros: this.ros,
      name: '/real/odom',
      messageType: 'nav_msgs/Odometry'
    });

    this.realOdomSubscriber.subscribe((message: any) => {
      try {
        const position = message.pose.pose.position;
        const orientation = message.pose.pose.orientation;
        // Approximate planar yaw from quaternion (assuming 2D)
        const theta = 2 * Math.atan2(orientation.z, orientation.w);
        const pose: RobotPose = {
          x: position.x,
          y: position.y,
          theta
        };
        this.notifyPoseChange(pose);
      } catch (e) {
        console.error('Failed to parse /real/odom message:', e);
      }
    });

    // Subscribe to battery state
    this.batterySubscriber = new Topic({
      ros: this.ros,
      name: '/battery_state',
      messageType: 'sensor_msgs/BatteryState'
    });

    this.batterySubscriber.subscribe((message: any) => {
      try {
        const battery = {
          percentage: message.percentage * 100, // Convert 0-1 to 0-100
          voltage: message.voltage
        };
        this.batteryCallbacks.forEach(cb => cb(battery));
      } catch (e) {
        console.error('Failed to parse /battery_state message:', e);
      }
    });

    // Subscribe to explanations
    this.explanationSubscriber = new Topic({
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

    // Subscribe to digital twin sensor diff
    this.twinSensorDiffSubscriber = new Topic({
      ros: this.ros,
      name: '/twin/sensor_diff',
      messageType: 'std_msgs/String'
    });

    this.twinSensorDiffSubscriber.subscribe((message: any) => {
      try {
        const diff: TwinSensorDiff = JSON.parse(message.data);
        this.twinSensorDiffCallbacks.forEach(cb => cb(diff));
      } catch (e) {
        console.error('Failed to parse twin sensor diff:', e);
      }
    });

    // Subscribe to anomaly score
    this.anomalyScoreSubscriber = new Topic({
      ros: this.ros,
      name: '/anomaly/score',
      messageType: 'std_msgs/Float32'
    });

    this.anomalyScoreSubscriber.subscribe((message: any) => {
      const score = typeof message.data === 'number' ? message.data : Number(message.data);
      if (!Number.isNaN(score)) {
        this.anomalyScoreCallbacks.forEach(cb => cb(score));
      }
    });

    // Subscribe to anomaly alerts
    this.anomalyAlertSubscriber = new Topic({
      ros: this.ros,
      name: '/anomaly/alert',
      messageType: 'std_msgs/String'
    });

    this.anomalyAlertSubscriber.subscribe((message: any) => {
      try {
        const alert: AnomalyAlert = JSON.parse(message.data);
        this.anomalyAlertCallbacks.forEach(cb => cb(alert));
      } catch (e) {
        console.error('Failed to parse anomaly alert:', e);
      }
    });
  }

  /**
   * Register callback for explanation updates
   */
  onExplanationReceived(callback: (explanation: Explanation) => void): void {
    this.explanationCallbacks.push(callback);
  }

  onTwinSensorDiff(callback: (diff: TwinSensorDiff) => void): void {
    this.twinSensorDiffCallbacks.push(callback);
  }

  onAnomalyScore(callback: (score: number) => void): void {
    this.anomalyScoreCallbacks.push(callback);
  }

  onAnomalyAlert(callback: (alert: AnomalyAlert) => void): void {
    this.anomalyAlertCallbacks.push(callback);
  }

  onBattery(callback: (battery: { percentage: number; voltage: number }) => void): void {
    this.batteryCallbacks.push(callback);
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

    const twist = {
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
    };

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

  /**
   * Get current rosbridge URL
   */
  getRosbridgeUrl(): string {
    return this.rosbridgeUrl;
  }
}

// Singleton instance
export const rosService = new ROSService();
