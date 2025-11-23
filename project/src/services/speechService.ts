/**
 * Speech Service with Backend Whisper API Integration
 *
 * Professional speech-to-text using OpenAI Whisper API via backend server.
 * Replaces browser Web Speech API with server-side transcription.
 *
 * Architecture:
 * Browser → MediaRecorder → Audio Blob → Backend → Whisper API → Transcript
 */

import { audioRecorderService, type RecordingResult } from './audioRecorderService';
import { backendService, type VoiceCommandResult } from './backendService';

export interface SpeechRecognitionResult {
  transcript: string;
  confidence: number;
  isFinal: boolean;
}

export interface CommandExecutionResult extends VoiceCommandResult {
  // Extends VoiceCommandResult with any additional frontend-specific fields
}

class SpeechService {
  private isListening = false;
  private isSupported = false;
  private useTurtlesim = false; // Toggle for turtlesim vs real robot
  private useCompletePipeline = true; // Use complete pipeline (Whisper → Gemini → Robot)

  private resultCallbacks: ((result: SpeechRecognitionResult) => void)[] = [];
  private commandExecutionCallbacks: ((result: CommandExecutionResult) => void)[] = [];
  private errorCallbacks: ((error: string) => void)[] = [];
  private statusCallbacks: ((status: 'idle' | 'listening' | 'processing' | 'executing') => void)[] = [];

  constructor() {
    this.checkSupport();
    this.setupRecording();
  }

  /**
   * Check if browser supports audio recording
   */
  private checkSupport(): void {
    this.isSupported = audioRecorderService.isSupported();

    if (!this.isSupported) {
      console.warn('Audio recording not supported. Requires modern browser with MediaRecorder API.');
    }
  }

  /**
   * Setup audio recording callbacks
   */
  private setupRecording(): void {
    // Handle recording completion
    audioRecorderService.onRecordingComplete(async (result: RecordingResult) => {
      this.notifyStatus('processing');
      await this.transcribeAudio(result);
    });

    // Handle recording errors
    audioRecorderService.onError((error: string) => {
      this.notifyError(error);
      this.isListening = false;
      this.notifyStatus('idle');
    });
  }

  /**
   * Transcribe recorded audio using backend Whisper API
   * If useCompletePipeline is true, also parses and executes the command
   */
  private async transcribeAudio(recording: RecordingResult): Promise<void> {
    try {
      console.log(`Transcribing ${recording.duration.toFixed(1)}s audio (${recording.size} bytes)`);

      // Check backend health first
      try {
        await backendService.checkHealth();
      } catch (error) {
        throw new Error('Backend server not available. Please start the backend server.');
      }

      // Send audio to backend for transcription
      const transcriptionResponse = await backendService.transcribeAudio(recording.audioBlob);
      console.log('Whisper API response:', transcriptionResponse);

      // Always notify transcription result
      this.notifyResult({
        transcript: transcriptionResponse.transcript,
        confidence: transcriptionResponse.confidence,
        isFinal: true
      });

      // If using complete pipeline, execute the command
      if (this.useCompletePipeline) {
        this.notifyStatus('executing');
        console.log('Executing command via complete pipeline...');

        const executionResult = await backendService.executeVoiceCommand(
          transcriptionResponse.transcript,
          this.useTurtlesim
        );

        console.log('Command execution result:', executionResult);

        // Notify command execution result
        this.notifyCommandExecution(executionResult);
      }

      this.notifyStatus('idle');

    } catch (error: any) {
      console.error('Voice command failed:', error);
      this.notifyError(error.message || 'Voice command failed');
      this.notifyStatus('idle');
    }
  }

  /**
   * Start listening for voice input (records audio for backend transcription)
   */
  async startListening(): Promise<void> {
    if (!this.isSupported) {
      this.notifyError('Audio recording not supported in this browser');
      return;
    }

    if (this.isListening) {
      console.warn('Already listening');
      return;
    }

    try {
      this.isListening = true;
      this.notifyStatus('listening');
      await audioRecorderService.startRecording();
      console.log('Recording started for Whisper transcription');
    } catch (error: any) {
      console.error('Failed to start recording:', error);
      this.notifyError(error.message || 'Failed to start microphone');
      this.isListening = false;
      this.notifyStatus('idle');
    }
  }

  /**
   * Stop listening (stops recording and sends to backend)
   */
  stopListening(): void {
    if (!this.isListening) {
      return;
    }

    try {
      audioRecorderService.stopRecording();
      this.isListening = false;
      console.log('Recording stopped, processing...');
    } catch (error: any) {
      console.error('Failed to stop recording:', error);
      this.notifyError('Failed to stop recording');
      this.isListening = false;
      this.notifyStatus('idle');
    }
  }

  /**
   * Toggle listening state
   */
  toggleListening(): void {
    if (this.isListening) {
      this.stopListening();
    } else {
      this.startListening();
    }
  }

  /**
   * Check if backend server is available
   */
  async checkBackendConnection(): Promise<boolean> {
    try {
      await backendService.checkHealth();
      return true;
    } catch (error) {
      return false;
    }
  }

  /**
   * Get backend server URL
   */
  getBackendUrl(): string {
    return backendService.getBaseUrl();
  }

  /**
   * Register callback for recognition results
   */
  onResult(callback: (result: SpeechRecognitionResult) => void): void {
    this.resultCallbacks.push(callback);
  }

  /**
   * Register callback for command execution results
   */
  onCommandExecution(callback: (result: CommandExecutionResult) => void): void {
    this.commandExecutionCallbacks.push(callback);
  }

  /**
   * Register callback for errors
   */
  onError(callback: (error: string) => void): void {
    this.errorCallbacks.push(callback);
  }

  /**
   * Register callback for status changes
   */
  onStatusChange(callback: (status: 'idle' | 'listening' | 'processing' | 'executing') => void): void {
    this.statusCallbacks.push(callback);
  }

  /**
   * Notify all result callbacks
   */
  private notifyResult(result: SpeechRecognitionResult): void {
    this.resultCallbacks.forEach(cb => cb(result));
  }

  /**
   * Notify all command execution callbacks
   */
  private notifyCommandExecution(result: CommandExecutionResult): void {
    this.commandExecutionCallbacks.forEach(cb => cb(result));
  }

  /**
   * Notify all error callbacks
   */
  private notifyError(error: string): void {
    this.errorCallbacks.forEach(cb => cb(error));
  }

  /**
   * Notify all status callbacks
   */
  private notifyStatus(status: 'idle' | 'listening' | 'processing' | 'executing'): void {
    this.statusCallbacks.forEach(cb => cb(status));
  }

  /**
   * Check if browser supports speech recognition
   */
  isBrowserSupported(): boolean {
    return this.isSupported;
  }

  /**
   * Check if currently listening
   */
  getIsListening(): boolean {
    return this.isListening;
  }

  /**
   * Set whether to use turtlesim or real robot topics
   */
  setUseTurtlesim(useTurtlesim: boolean): void {
    this.useTurtlesim = useTurtlesim;
    console.log(`Using ${useTurtlesim ? 'turtlesim' : 'real robot'} topics`);
  }

  /**
   * Get current turtlesim setting
   */
  getUseTurtlesim(): boolean {
    return this.useTurtlesim;
  }

  /**
   * Set whether to use complete pipeline (transcription + parsing + execution)
   * or just transcription
   */
  setUseCompletePipeline(useCompletePipeline: boolean): void {
    this.useCompletePipeline = useCompletePipeline;
    console.log(`Complete pipeline: ${useCompletePipeline ? 'enabled' : 'disabled'}`);
  }

  /**
   * Get current pipeline mode
   */
  getUseCompletePipeline(): boolean {
    return this.useCompletePipeline;
  }

  /**
   * Send direct twist command to robot
   */
  async sendTwistCommand(linearX: number, angularZ: number): Promise<void> {
    try {
      await backendService.sendTwistCommand(linearX, angularZ, this.useTurtlesim);
    } catch (error: any) {
      this.notifyError(error.message || 'Failed to send twist command');
      throw error;
    }
  }

  /**
   * Stop the robot immediately
   */
  async stopRobot(): Promise<void> {
    try {
      await backendService.stopRobot(this.useTurtlesim);
    } catch (error: any) {
      this.notifyError(error.message || 'Failed to stop robot');
      throw error;
    }
  }

  /**
   * Get current robot state
   */
  async getRobotState(): Promise<any> {
    try {
      return await backendService.getRobotState();
    } catch (error: any) {
      this.notifyError(error.message || 'Failed to get robot state');
      throw error;
    }
  }
}

// Singleton instance
export const speechService = new SpeechService();
