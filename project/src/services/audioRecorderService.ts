/**
 * Audio Recorder Service for Backend Whisper API Integration
 *
 * Captures audio from microphone and sends to backend for transcription.
 * Replaces browser Web Speech API with professional Whisper API.
 */

export interface AudioRecordingOptions {
  sampleRate?: number;
  channelCount?: number;
  mimeType?: string;
}

export interface RecordingResult {
  audioBlob: Blob;
  duration: number;
  size: number;
}

class AudioRecorderService {
  private mediaRecorder: MediaRecorder | null = null;
  private audioChunks: Blob[] = [];
  private stream: MediaStream | null = null;
  private startTime: number = 0;
  private isRecording: boolean = false;
  private recordingCallbacks: ((result: RecordingResult) => void)[] = [];
  private errorCallbacks: ((error: string) => void)[] = [];

  /**
   * Check if browser supports audio recording
   */
  isSupported(): boolean {
    return !!(navigator.mediaDevices && navigator.mediaDevices.getUserMedia && window.MediaRecorder);
  }

  /**
   * Request microphone permission and start recording
   */
  async startRecording(options: AudioRecordingOptions = {}): Promise<void> {
    if (!this.isSupported()) {
      const error = 'Audio recording not supported in this browser';
      this.notifyError(error);
      throw new Error(error);
    }

    if (this.isRecording) {
      console.warn('Already recording');
      return;
    }

    try {
      // Request microphone access
      this.stream = await navigator.mediaDevices.getUserMedia({
        audio: {
          sampleRate: options.sampleRate || 16000, // Whisper optimal: 16kHz
          channelCount: options.channelCount || 1,  // Mono
          echoCancellation: true,
          noiseSuppression: true,
          autoGainControl: true
        }
      });

      // Determine MIME type (browser-dependent)
      const mimeType = this.getSupportedMimeType(options.mimeType);

      // Create MediaRecorder
      this.mediaRecorder = new MediaRecorder(this.stream, {
        mimeType: mimeType,
        audioBitsPerSecond: 128000 // 128 kbps
      });

      // Reset chunks
      this.audioChunks = [];
      this.startTime = Date.now();

      // Handle data available
      this.mediaRecorder.ondataavailable = (event) => {
        if (event.data.size > 0) {
          this.audioChunks.push(event.data);
        }
      };

      // Handle recording stop
      this.mediaRecorder.onstop = () => {
        const duration = (Date.now() - this.startTime) / 1000; // seconds
        const audioBlob = new Blob(this.audioChunks, { type: mimeType });

        const result: RecordingResult = {
          audioBlob,
          duration,
          size: audioBlob.size
        };

        this.notifyRecordingComplete(result);
        this.cleanup();
      };

      // Handle errors
      this.mediaRecorder.onerror = (event: any) => {
        const error = `Recording error: ${event.error?.message || 'Unknown error'}`;
        console.error(error);
        this.notifyError(error);
        this.cleanup();
      };

      // Start recording
      this.mediaRecorder.start();
      this.isRecording = true;
      console.log(`Recording started with ${mimeType}`);

    } catch (error: any) {
      const errorMsg = error.name === 'NotAllowedError'
        ? 'Microphone permission denied'
        : `Failed to start recording: ${error.message}`;

      this.notifyError(errorMsg);
      this.cleanup();
      throw new Error(errorMsg);
    }
  }

  /**
   * Stop recording and get audio blob
   */
  stopRecording(): void {
    if (!this.isRecording || !this.mediaRecorder) {
      console.warn('Not currently recording');
      return;
    }

    this.mediaRecorder.stop();
    this.isRecording = false;
    console.log('Recording stopped');
  }

  /**
   * Get supported MIME type for this browser
   */
  private getSupportedMimeType(preferred?: string): string {
    if (preferred && MediaRecorder.isTypeSupported(preferred)) {
      return preferred;
    }

    const types = [
      'audio/webm;codecs=opus',
      'audio/webm',
      'audio/ogg;codecs=opus',
      'audio/mp4',
      'audio/mpeg'
    ];

    for (const type of types) {
      if (MediaRecorder.isTypeSupported(type)) {
        return type;
      }
    }

    return 'audio/webm'; // Fallback
  }

  /**
   * Cleanup resources
   */
  private cleanup(): void {
    if (this.stream) {
      this.stream.getTracks().forEach(track => track.stop());
      this.stream = null;
    }
    this.mediaRecorder = null;
    this.isRecording = false;
  }

  /**
   * Check if currently recording
   */
  getIsRecording(): boolean {
    return this.isRecording;
  }

  /**
   * Register callback for recording completion
   */
  onRecordingComplete(callback: (result: RecordingResult) => void): void {
    this.recordingCallbacks.push(callback);
  }

  /**
   * Register callback for errors
   */
  onError(callback: (error: string) => void): void {
    this.errorCallbacks.push(callback);
  }

  /**
   * Notify recording complete callbacks
   */
  private notifyRecordingComplete(result: RecordingResult): void {
    this.recordingCallbacks.forEach(cb => cb(result));
  }

  /**
   * Notify error callbacks
   */
  private notifyError(error: string): void {
    this.errorCallbacks.forEach(cb => cb(error));
  }

  /**
   * Convert audio blob to base64 (for API transmission if needed)
   */
  async blobToBase64(blob: Blob): Promise<string> {
    return new Promise((resolve, reject) => {
      const reader = new FileReader();
      reader.onloadend = () => {
        const base64 = reader.result as string;
        resolve(base64.split(',')[1]); // Remove data:audio/...;base64, prefix
      };
      reader.onerror = reject;
      reader.readAsDataURL(blob);
    });
  }

  /**
   * Get audio level (for visualization)
   */
  getAudioLevel(): number {
    // TODO: Implement audio level detection using AnalyserNode
    // This would require connecting AudioContext to the stream
    return 0;
  }
}

// Singleton instance
export const audioRecorderService = new AudioRecorderService();
