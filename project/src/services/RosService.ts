import * as ROSLIB from 'roslib';

class RosService {
  private ros: ROSLIB.Ros;
  private connected: boolean = false;
  private subscribers: Map<string, ROSLIB.Topic> = new Map();

  constructor() {
    this.ros = new ROSLIB.Ros({});
  }

  connect(url: string = 'ws://localhost:9090'): Promise<boolean> {
    return new Promise((resolve, reject) => {
      this.ros.connect(url);

      this.ros.on('connection', () => {
        console.log('Connected to ROS bridge server');
        this.connected = true;
        resolve(true);
      });

      this.ros.on('error', (error: any) => {
        console.error('Error connecting to ROS bridge server:', error);
        this.connected = false;
        reject(error);
      });

      this.ros.on('close', () => {
        console.log('Connection to ROS bridge server closed');
        this.connected = false;
      });
    });
  }

  disconnect(): void {
    if (this.connected) {
      this.ros.close();
      this.connected = false;
    }
  }

  isConnected(): boolean {
    return this.connected;
  }

  // Publish a message to a topic
  publish(topicName: string, messageType: string, message: any): void {
    if (!this.connected) {
      console.error('Not connected to ROS bridge server');
      return;
    }

    const topic = new ROSLIB.Topic({
      ros: this.ros,
      name: topicName,
      messageType: messageType
    });

    const rosMessage = new ROSLIB.Message(message);
    topic.publish(rosMessage);
  }

  // Subscribe to a topic with a callback
  subscribe(topicName: string, messageType: string, callback: (message: any) => void): void {
    if (!this.connected) {
      console.error('Not connected to ROS bridge server');
      return;
    }

    // Check if we already have a subscriber for this topic
    if (this.subscribers.has(topicName)) {
      // Unsubscribe first
      this.subscribers.get(topicName)?.unsubscribe();
    }

    const topic = new ROSLIB.Topic({
      ros: this.ros,
      name: topicName,
      messageType: messageType
    });

    topic.subscribe(callback);
    this.subscribers.set(topicName, topic);
  }

  // Unsubscribe from a topic
  unsubscribe(topicName: string): void {
    if (this.subscribers.has(topicName)) {
      this.subscribers.get(topicName)?.unsubscribe();
      this.subscribers.delete(topicName);
    }
  }

  // Call a service
  callService(serviceName: string, serviceType: string, request: any): Promise<any> {
    return new Promise((resolve, reject) => {
      if (!this.connected) {
        reject('Not connected to ROS bridge server');
        return;
      }

      const service = new ROSLIB.Service({
        ros: this.ros,
        name: serviceName,
        serviceType: serviceType
      });

      const serviceRequest = new ROSLIB.ServiceRequest(request);

      service.callService(serviceRequest, (result) => {
        resolve(result);
      }, (error) => {
        reject(error);
      });
    });
  }

  // Set values for circular motion
  setCircularMotion(linearSpeed: number, angularSpeed: number, active: boolean = true): void {
    // Publish linear speed
    this.publish('/turtle1/linear_speed', 'std_msgs/Float32', { data: linearSpeed });
    
    // Publish angular speed
    this.publish('/turtle1/angular_speed', 'std_msgs/Float32', { data: angularSpeed });
    
    // Publish motion active state
    this.publish('/turtle1/motion_active', 'std_msgs/Bool', { data: active });
  }
  
  // Stop motion
  stopMotion(): void {
    this.publish('/turtle1/motion_active', 'std_msgs/Bool', { data: false });
  }
}

// Create a singleton instance
const rosService = new RosService();
export default rosService; 