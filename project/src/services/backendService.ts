/**
 * Backend API Service
 *
 * Handles communication with FastAPI backend server.
 * Provides methods for:
 * - Audio transcription (Whisper API)
 * - Command parsing (Gemini)
 * - WebSocket communication
 */

export interface TranscriptionResponse {
  transcript: string;
  language: string;
  confidence: number;
  duration: number;
}

export interface ParsedCommand {
  action: string;
  parameters?: any;
  confidence: number;
  reasoning?: string;
}

export interface VoiceCommandResult {
  transcript: string;
  parsed_command: ParsedCommand;
  execution_status: string;
  session_id: string;  // NEW: Session ID for conversation memory
  resolved_reference: string | null;  // NEW: Resolved spatial reference description
  latency: {
    gemini_ms: number;
    execution_ms: number;
    memory_ms: number;  // NEW: Memory operation latency
    total_ms: number;
  };
}

export interface BackendConfig {
  baseUrl: string;
  timeout?: number;
}

class BackendService {
  private baseUrl: string;
  private timeout: number;

  constructor(config?: BackendConfig) {
    this.baseUrl = config?.baseUrl || import.meta.env.VITE_BACKEND_URL || 'http://localhost:8000';
    this.timeout = config?.timeout || 30000; // 30 seconds
  }

  /**
   * Check backend server health
   */
  async checkHealth(): Promise<{ status: string; timestamp: string }> {
    try {
      const response = await fetch(`${this.baseUrl}/health`, {
        method: 'GET',
        signal: AbortSignal.timeout(5000) // 5 second timeout for health check
      });

      if (!response.ok) {
        throw new Error(`Health check failed: ${response.status}`);
      }

      return await response.json();
    } catch (error: any) {
      console.error('Backend health check failed:', error);
      throw new Error(`Backend server unreachable: ${error.message}`);
    }
  }

  /**
   * Transcribe audio using Whisper API
   *
   * @param audioBlob - Audio file blob
   * @returns Transcription result
   */
  async transcribeAudio(audioBlob: Blob): Promise<TranscriptionResponse> {
    try {
      const formData = new FormData();

      // Convert blob to file with proper name and type
      const audioFile = new File([audioBlob], 'audio.webm', {
        type: audioBlob.type || 'audio/webm'
      });

      formData.append('audio', audioFile);

      console.log(`Sending audio to backend: ${audioBlob.size} bytes, type: ${audioBlob.type}`);

      const response = await fetch(`${this.baseUrl}/api/v1/transcribe`, {
        method: 'POST',
        body: formData,
        signal: AbortSignal.timeout(this.timeout)
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.detail || `Transcription failed: ${response.status}`);
      }

      const result = await response.json();
      console.log('Transcription successful:', result);

      return result;
    } catch (error: any) {
      console.error('Transcription request failed:', error);

      if (error.name === 'AbortError' || error.name === 'TimeoutError') {
        throw new Error('Transcription timeout - audio may be too long');
      }

      throw new Error(`Transcription failed: ${error.message}`);
    }
  }

  /**
   * Parse natural language command
   *
   * @param command - User command text
   * @param context - Optional robot state/context
   * @returns Parsed command structure
   */
  async parseCommand(command: string, context?: any): Promise<ParsedCommand> {
    try {
      const response = await fetch(`${this.baseUrl}/api/v1/parse`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({
          command,
          context: context || {}
        }),
        signal: AbortSignal.timeout(10000) // 10 second timeout
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.detail || `Parsing failed: ${response.status}`);
      }

      return await response.json();
    } catch (error: any) {
      console.error('Command parsing failed:', error);
      throw new Error(`Failed to parse command: ${error.message}`);
    }
  }

  /**
   * Execute complete voice command pipeline
   * (Transcription → Parsing → Robot Execution)
   *
   * @param transcript - Voice command text from Whisper
   * @param useTurtlesim - Whether to use turtlesim topics
   * @param context - Optional robot state/context
   * @returns Complete execution result with latency metrics
   */
  async executeVoiceCommand(
    transcript: string,
    useTurtlesim: boolean = false,
    sessionId?: string,
    context?: any
  ): Promise<VoiceCommandResult> {
    try {
      const requestBody: any = {
        transcript,
        use_turtlesim: useTurtlesim,
        context: context || {}
      };

      // Include session_id if provided
      if (sessionId) {
        requestBody.session_id = sessionId;
      }

      const response = await fetch(`${this.baseUrl}/api/v1/execute_voice_command`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify(requestBody),
        signal: AbortSignal.timeout(15000) // 15 second timeout
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.detail || `Execution failed: ${response.status}`);
      }

      const result = await response.json();
      console.log('Voice command executed:', result);

      return result;
    } catch (error: any) {
      console.error('Voice command execution failed:', error);

      if (error.name === 'AbortError' || error.name === 'TimeoutError') {
        throw new Error('Command execution timeout');
      }

      throw new Error(`Failed to execute command: ${error.message}`);
    }
  }

  /**
   * Send robot control command (twist)
   *
   * @param linearX - Linear velocity (m/s)
   * @param angularZ - Angular velocity (rad/s)
   * @param useTurtlesim - Whether to use turtlesim topics
   */
  async sendTwistCommand(
    linearX: number,
    angularZ: number,
    useTurtlesim: boolean = false
  ): Promise<void> {
    try {
      const response = await fetch(`${this.baseUrl}/api/v1/robot/twist`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({
          linear_x: linearX,
          angular_z: angularZ,
          use_turtlesim: useTurtlesim
        }),
        signal: AbortSignal.timeout(5000)
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.detail || `Robot control failed: ${response.status}`);
      }
    } catch (error: any) {
      console.error('Twist command failed:', error);
      throw new Error(`Failed to send robot command: ${error.message}`);
    }
  }

  /**
   * Stop the robot
   *
   * @param useTurtlesim - Whether to use turtlesim topics
   */
  async stopRobot(useTurtlesim: boolean = false): Promise<void> {
    try {
      const response = await fetch(`${this.baseUrl}/api/v1/robot/stop`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({
          use_turtlesim: useTurtlesim
        }),
        signal: AbortSignal.timeout(5000)
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.detail || `Stop command failed: ${response.status}`);
      }
    } catch (error: any) {
      console.error('Stop command failed:', error);
      throw new Error(`Failed to stop robot: ${error.message}`);
    }
  }

  /**
   * Get robot state (pose, velocity)
   */
  async getRobotState(): Promise<any> {
    try {
      const response = await fetch(`${this.baseUrl}/api/v1/robot/state`, {
        method: 'GET',
        signal: AbortSignal.timeout(5000)
      });

      if (!response.ok) {
        throw new Error(`Failed to get robot state: ${response.status}`);
      }

      return await response.json();
    } catch (error: any) {
      console.error('Failed to get robot state:', error);
      throw new Error(`Failed to get robot state: ${error.message}`);
    }
  }

  /**
   * Get backend base URL
   */
  getBaseUrl(): string {
    return this.baseUrl;
  }

  /**
   * Set backend base URL
   */
  setBaseUrl(url: string): void {
    this.baseUrl = url;
  }
}

// Singleton instance
export const backendService = new BackendService();
