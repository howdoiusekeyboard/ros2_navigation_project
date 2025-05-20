declare module 'roslib' {
  export interface RosOptions {
    url?: string;
    groovyCompatibility?: boolean;
    transportLibrary?: string;
  }

  export class Ros {
    constructor(options?: RosOptions);
    connect(url: string): void;
    close(): void;
    on(eventName: string, callback: (event?: any) => void): void;
  }

  export interface TopicOptions {
    ros: Ros;
    name: string;
    messageType: string;
    compression?: string;
    throttle_rate?: number;
    queue_size?: number;
    latch?: boolean;
  }

  export class Topic {
    constructor(options: TopicOptions);
    subscribe(callback: (message: any) => void): void;
    unsubscribe(): void;
    publish(message: Message): void;
  }

  export class Message {
    constructor(values: any);
  }

  export interface ServiceOptions {
    ros: Ros;
    name: string;
    serviceType: string;
  }

  export class Service {
    constructor(options: ServiceOptions);
    callService(
      request: ServiceRequest, 
      callback: (response: any) => void, 
      failedCallback?: (error: any) => void
    ): void;
  }

  export class ServiceRequest {
    constructor(values: any);
  }

  export class ServiceResponse {
    constructor(values: any);
  }

  export interface ActionClientOptions {
    ros: Ros;
    serverName: string;
    actionName: string;
    timeout?: number;
    omitFeedback?: boolean;
    omitStatus?: boolean;
    omitResult?: boolean;
  }

  export class ActionClient {
    constructor(options: ActionClientOptions);
    goalMessage: any;
    cancel(): void;
    on(eventName: string, callback: (event?: any) => void): void;
    sendGoal(goal: any): void;
  }

  export default {
    Ros,
    Topic,
    Message,
    Service,
    ServiceRequest,
    ServiceResponse,
    ActionClient
  };
} 