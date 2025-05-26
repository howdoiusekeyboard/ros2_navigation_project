/*
 * Centralized Turtle Control Service
 * This service acts as the single source of truth for all turtle motion control.
 * It ensures perfect synchronization between sliders, command input, and voice commands.
 */

import rosService from './RosService';

export interface TurtleState {
  linearSpeed: number;
  angularSpeed: number;
  isMoving: boolean;
  connected: boolean;
}

export interface CommandHistoryItem {
  id: string;
  command: string;
  timestamp: Date;
  result: 'success' | 'error';
  parsedAction?: {
    action: 'start' | 'stop';
    linear?: number;
    angular?: number;
  };
}

type StateChangeListener = (state: TurtleState) => void;
type CommandHistoryListener = (history: CommandHistoryItem[]) => void;

class TurtleControlService {
  private state: TurtleState = {
    linearSpeed: 2.0,
    angularSpeed: 1.0,
    isMoving: false,
    connected: false
  };

  private commandHistory: CommandHistoryItem[] = [];
  private publishTimer: NodeJS.Timeout | null = null;
  private stateListeners: Set<StateChangeListener> = new Set();
  private historyListeners: Set<CommandHistoryListener> = new Set();

  constructor() {
    this.initializeRosConnection();
  }

  // State management
  getState(): TurtleState {
    return { ...this.state };
  }

  onStateChange(listener: StateChangeListener): () => void {
    this.stateListeners.add(listener);
    return () => this.stateListeners.delete(listener);
  }

  onHistoryChange(listener: CommandHistoryListener): () => void {
    this.historyListeners.add(listener);
    return () => this.historyListeners.delete(listener);
  }

  private notifyStateChange(): void {
    this.stateListeners.forEach(listener => listener(this.getState()));
  }

  private notifyHistoryChange(): void {
    this.historyListeners.forEach(listener => listener([...this.commandHistory]));
  }

  // ROS connection management
  private async initializeRosConnection(): Promise<void> {
    try {
      const connected = await rosService.connect();
      this.state.connected = connected;
      this.notifyStateChange();
    } catch (error) {
      console.error('Failed to connect to ROS:', error);
      this.state.connected = false;
      this.notifyStateChange();
    }
  }

  async reconnect(): Promise<boolean> {
    try {
      const connected = await rosService.connect();
      this.state.connected = connected;
      this.notifyStateChange();
      return connected;
    } catch (error) {
      console.error('Failed to reconnect to ROS:', error);
      this.state.connected = false;
      this.notifyStateChange();
      return false;
    }
  }

  // Motion control - Single source of truth
  setSpeed(linear: number, angular: number, source: 'slider' | 'command' | 'voice' = 'slider'): void {
    this.state.linearSpeed = linear;
    this.state.angularSpeed = angular;
    
    // If currently moving, immediately publish the new speeds
    if (this.state.isMoving) {
      this.publishCurrentState();
    }
    
    this.notifyStateChange();
    console.log(`Speed updated from ${source}: linear=${linear}, angular=${angular}`);
  }

  start(source: 'manual' | 'command' | 'voice' = 'manual'): void {
    if (!this.state.connected) {
      console.warn('Cannot start: not connected to ROS');
      return;
    }

    this.state.isMoving = true;
    this.startPublishing();
    this.notifyStateChange();
    console.log(`Motion started from ${source}`);
  }

  stop(source: 'manual' | 'command' | 'voice' = 'manual'): void {
    this.state.isMoving = false;
    this.stopPublishing();
    this.notifyStateChange();
    console.log(`Motion stopped from ${source}`);
  }

  private startPublishing(): void {
    // Clear any existing timer
    if (this.publishTimer) {
      clearInterval(this.publishTimer);
    }

    // Publish immediately
    this.publishCurrentState();

    // Continue publishing at 10Hz
    this.publishTimer = setInterval(() => {
      this.publishCurrentState();
    }, 100);
  }

  private stopPublishing(): void {
    if (this.publishTimer) {
      clearInterval(this.publishTimer);
      this.publishTimer = null;
    }
    
    // Send stop command
    rosService.stopMotion();
  }

  private publishCurrentState(): void {
    if (this.state.connected) {
      rosService.setCircularMotion(
        this.state.linearSpeed, 
        this.state.angularSpeed, 
        this.state.isMoving
      );
    }
  }

  // Command history management
  addToHistory(command: string, result: 'success' | 'error', parsedAction?: any): void {
    const historyItem: CommandHistoryItem = {
      id: Date.now().toString(),
      command,
      timestamp: new Date(),
      result,
      parsedAction
    };

    this.commandHistory.unshift(historyItem);
    
    // Keep only last 50 commands
    if (this.commandHistory.length > 50) {
      this.commandHistory = this.commandHistory.slice(0, 50);
    }

    this.notifyHistoryChange();
  }

  executeHistoryItem(item: CommandHistoryItem): void {
    if (item.parsedAction) {
      this.executeAction(item.parsedAction, 'command');
      this.addToHistory(`Re-executed: ${item.command}`, 'success', item.parsedAction);
    }
  }

  executeAction(action: { action: 'start' | 'stop'; linear?: number; angular?: number }, source: 'command' | 'voice'): void {
    switch (action.action) {
      case 'start':
        if (action.linear !== undefined && action.angular !== undefined) {
          this.setSpeed(action.linear, action.angular, source);
        }
        this.start(source);
        break;
      case 'stop':
        this.stop(source);
        break;
    }
  }

  // Cleanup
  destroy(): void {
    this.stop();
    this.stateListeners.clear();
    this.historyListeners.clear();
  }
}

// Singleton instance
export const turtleControlService = new TurtleControlService(); 