import { GoogleGenerativeAI } from '@google/generative-ai';
import type { TwistCommand } from './rosService';

export interface ParsedCommand {
  action: 'twist' | 'navigate' | 'stop' | 'unknown';
  parameters?: TwistCommand | { x: number; y: number; theta: number };
  confidence: number;
  original: string;
}

class GeminiService {
  private genAI: GoogleGenerativeAI | null = null;
  private model: any = null;
  private isConfigured = false;

  /**
   * Initialize Gemini with API key
   */
  configure(apiKey: string): void {
    if (!apiKey) {
      console.error('Gemini API key is required');
      return;
    }

    try {
      this.genAI = new GoogleGenerativeAI(apiKey);
      this.model = this.genAI.getGenerativeModel({
        model: 'gemini-2.0-flash-exp',
        generationConfig: {
          temperature: 0.3, // Low temperature for consistent command parsing
          topP: 0.95,
          topK: 40,
          maxOutputTokens: 256,
        },
      });
      this.isConfigured = true;
      console.log('Gemini service configured successfully');
    } catch (error) {
      console.error('Failed to configure Gemini:', error);
      this.isConfigured = false;
    }
  }

  /**
   * Parse natural language command into structured robot command
   */
  async parseCommand(userCommand: string, robotContext?: { x: number; y: number; theta: number }): Promise<ParsedCommand> {
    if (!this.isConfigured || !this.model) {
      throw new Error('Gemini service not configured. Call configure() with API key first.');
    }

    const systemPrompt = this.buildSystemPrompt(robotContext);
    const fullPrompt = `${systemPrompt}\n\nUser command: "${userCommand}"\n\nParse this command and respond ONLY with valid JSON.`;

    try {
      const result = await this.model.generateContent(fullPrompt);
      const response = await result.response;
      const text = response.text();

      // Extract JSON from response (handle markdown code blocks)
      const jsonMatch = text.match(/\{[\s\S]*\}/);
      if (!jsonMatch) {
        throw new Error('No valid JSON found in response');
      }

      const parsed = JSON.parse(jsonMatch[0]);

      // Validate and normalize the response
      return this.validateCommand(parsed, userCommand);
    } catch (error) {
      console.error('Error parsing command with Gemini:', error);

      // Fallback: try simple pattern matching
      return this.fallbackParse(userCommand);
    }
  }

  /**
   * Build system prompt with robot capabilities and context
   */
  private buildSystemPrompt(context?: { x: number; y: number; theta: number }): string {
    const contextInfo = context
      ? `Current robot position: x=${context.x.toFixed(2)}, y=${context.y.toFixed(2)}, theta=${context.theta.toFixed(2)} radians`
      : 'Robot position unknown';

    return `You are a robot command parser. Convert natural language into structured JSON commands.

${contextInfo}

Robot capabilities:
1. TWIST: Move with linear and angular velocity (for spinning, moving forward/back, turning)
2. STOP: Immediately stop all motion
3. NAVIGATE: Go to specific coordinates (for TurtleBot3 with Nav2)

Output format (choose ONE):

For motion commands (spin, move, turn, circle):
{
  "action": "twist",
  "parameters": {
    "linear": <float 0.0-2.0>,  // m/s forward speed
    "angular": <float -1.0 to 1.0>  // rad/s rotation speed
  },
  "confidence": <float 0.0-1.0>
}

For stop commands:
{
  "action": "stop",
  "confidence": 1.0
}

For navigation commands (go to location):
{
  "action": "navigate",
  "parameters": {
    "x": <float>,
    "y": <float>,
    "theta": <float>  // orientation in radians
  },
  "confidence": <float 0.0-1.0>
}

For unclear commands:
{
  "action": "unknown",
  "confidence": 0.0
}

Rules:
- "spin/circle/rotate" → twist with angular velocity
- "move forward/back" → twist with linear velocity
- "stop/halt" → stop action
- "go to X" → navigate action
- Be conservative with speeds for safety
- Default linear speed: 0.5-1.0 m/s
- Default angular speed: 0.3-0.8 rad/s
- Confidence based on clarity of command`;
  }

  /**
   * Validate and normalize Gemini's response
   */
  private validateCommand(parsed: any, original: string): ParsedCommand {
    const command: ParsedCommand = {
      action: parsed.action || 'unknown',
      confidence: Math.min(Math.max(parsed.confidence || 0, 0), 1),
      original
    };

    // Validate parameters based on action type
    if (command.action === 'twist' && parsed.parameters) {
      command.parameters = {
        linear: this.clamp(parsed.parameters.linear || 0, -2.0, 2.0),
        angular: this.clamp(parsed.parameters.angular || 0, -1.0, 1.0)
      };
    } else if (command.action === 'navigate' && parsed.parameters) {
      command.parameters = {
        x: parsed.parameters.x || 0,
        y: parsed.parameters.y || 0,
        theta: parsed.parameters.theta || 0
      };
    }

    return command;
  }

  /**
   * Fallback parser using simple pattern matching (when Gemini fails)
   */
  private fallbackParse(command: string): ParsedCommand {
    const lower = command.toLowerCase();

    // Emergency stop
    if (lower.match(/\b(stop|halt|freeze)\b/)) {
      return {
        action: 'stop',
        confidence: 1.0,
        original: command
      };
    }

    // Spin/rotate/circle
    if (lower.match(/\b(spin|circle|rotate|turn around)\b/)) {
      const slow = lower.includes('slow');
      return {
        action: 'twist',
        parameters: {
          linear: slow ? 0.5 : 1.0,
          angular: slow ? 0.5 : 0.8
        },
        confidence: 0.7,
        original: command
      };
    }

    // Move forward/backward
    const forwardMatch = lower.match(/\b(forward|ahead)\b.*?(\d+(?:\.\d+)?)/);
    const backwardMatch = lower.match(/\b(back|backward|reverse)\b.*?(\d+(?:\.\d+)?)/);

    if (forwardMatch) {
      const speed = parseFloat(forwardMatch[2]) || 1.0;
      return {
        action: 'twist',
        parameters: {
          linear: this.clamp(speed, 0, 2.0),
          angular: 0
        },
        confidence: 0.8,
        original: command
      };
    }

    if (backwardMatch) {
      const speed = parseFloat(backwardMatch[2]) || 1.0;
      return {
        action: 'twist',
        parameters: {
          linear: -this.clamp(speed, 0, 2.0),
          angular: 0
        },
        confidence: 0.8,
        original: command
      };
    }

    // Unknown command
    return {
      action: 'unknown',
      confidence: 0.0,
      original: command
    };
  }

  /**
   * Clamp value between min and max
   */
  private clamp(value: number, min: number, max: number): number {
    return Math.min(Math.max(value, min), max);
  }

  /**
   * Check if service is ready
   */
  isReady(): boolean {
    return this.isConfigured;
  }
}

// Singleton instance
export const geminiService = new GeminiService();
