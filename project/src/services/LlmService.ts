/*
 * Very thin wrapper around an LLM endpoint (e.g. OpenAI, Ollama, etc.) that
 * converts natural-language instructions into a structured JSON plan that the
 * UI can execute. In this project we only use it to extract linear & angular
 * velocities for circular motion, but it can be extended easily.
 */

export interface ParsedMotionCommand {
  action: 'start' | 'stop';
  linear?: number;
  angular?: number;
}

class LlmService {
  private readonly apiKey = import.meta.env.VITE_GEMINI_API_KEY ?? '';
  private readonly apiBase = 'https://generativelanguage.googleapis.com/v1beta/models/gemini-2.0-flash:generateContent';

  async parseCircularMotion(text: string): Promise<ParsedMotionCommand | null> {
    if (!this.apiKey) {
      console.warn('GEMINI_API_KEY missing – falling back to local parser');
      return null;
    }

    const prompt = [
      'Convert this robot command to JSON format.',
      'For spinning/rotating/circular motion: {"action":"start","linear":<m/s>,"angular":<rad/s>}',
      'For stopping: {"action":"stop"}',
      'Use reasonable speed values (linear: 0.5-3.0, angular: 0.5-2.0).',
      'Return ONLY valid JSON, no explanation.',
      '',
      `Command: "${text}"`
    ].join('\n');

    const body = {
      contents: [
        {
          parts: [
            {
              text: prompt
            }
          ]
        }
      ],
      generationConfig: {
        temperature: 0.1,
        maxOutputTokens: 100,
      }
    };

    try {
      const res = await fetch(`${this.apiBase}?key=${this.apiKey}`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(body),
      });

      if (!res.ok) {
        console.error('Gemini API error', res.status, res.statusText);
        return null;
      }

      const data = await res.json();
      const answer = data.candidates?.[0]?.content?.parts?.[0]?.text ?? '';

      // Clean the response and try to extract JSON
      const jsonMatch = answer.match(/\{[^}]+\}/);
      if (!jsonMatch) {
        console.error('No JSON found in Gemini response:', answer);
        return null;
      }

      return JSON.parse(jsonMatch[0]) as ParsedMotionCommand;
    } catch (e) {
      console.error('Failed to parse Gemini response', e);
      return null;
    }
  }
}

export const llmService = new LlmService(); 