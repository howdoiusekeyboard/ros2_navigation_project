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
import { backendService } from './backendService';

export interface SpeechRecognitionResult {
  transcript: string;
  confidence: number;
  isFinal: boolean;
}

class SpeechService {
  private isListening = false;
  private isSupported = false;
  private resultCallbacks: ((result: SpeechRecognitionResult) => void)[] = [];
  private errorCallbacks: ((error: string) => void)[] = [];
  private statusCallbacks: ((status: 'idle' | 'listening' | 'processing') => void)[] = [];

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
      const response = await backendService.transcribeAudio(recording.audioBlob);

      console.log('Whisper API response:', response);

      // Notify result (always final with Whisper)
      this.notifyResult({
        transcript: response.transcript,
        confidence: response.confidence,
        isFinal: true
      });

      this.notifyStatus('idle');

    } catch (error: any) {
      console.error('Transcription failed:', error);
      this.notifyError(error.message || 'Transcription failed');
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
   * Register callback for errors
   */
  onError(callback: (error: string) => void): void {
    this.errorCallbacks.push(callback);
  }

  /**
   * Register callback for status changes
   */
  onStatusChange(callback: (status: 'idle' | 'listening' | 'processing') => void): void {
    this.statusCallbacks.push(callback);
  }

  /**
   * Notify all result callbacks
   */
  private notifyResult(result: SpeechRecognitionResult): void {
    this.resultCallbacks.forEach(cb => cb(result));
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
  private notifyStatus(status: 'idle' | 'listening' | 'processing'): void {
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
}

// Singleton instance
export const speechService = new SpeechService();
