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
