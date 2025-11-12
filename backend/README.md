# Voice-Controlled Robot Backend Server

FastAPI backend for professional voice control with Whisper API and Gemini integration.

## Architecture

```
Frontend (React) → Backend (FastAPI) → ROS2 (rosbridge)
                       ↓
                  Whisper API (OpenAI)
                       ↓
                  Gemini API (Google)
                       ↓
                  Database (SQLite/PostgreSQL)
```

## Features

- **Whisper API Integration:** Professional speech-to-text (replaces Web Speech API)
- **Gemini Command Parsing:** Server-side LLM processing
- **ROS2 Integration:** Control Nav2 navigation stack
- **Context Management:** Database for conversation history
- **WebSocket Support:** Real-time audio streaming
- **RESTful API:** Well-documented endpoints

## Setup

### 1. Install Python Dependencies

```bash
cd backend
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
```

### 2. Get API Keys

**OpenAI (for Whisper):**
1. Go to: https://platform.openai.com/api-keys
2. Create new API key
3. Copy key

**Google Gemini:**
1. Go to: https://aistudio.google.com/app/apikey
2. Create new API key
3. Copy key

### 3. Configure Environment

Edit `.env` file:

```env
OPENAI_API_KEY=your_openai_key_here
GEMINI_API_KEY=your_gemini_key_here
```

### 4. Run Server

```bash
# Development mode (auto-reload)
python3 -m uvicorn app.main:app --reload --host 0.0.0.0 --port 8000

# Or using the app directly
python3 app/main.py
```

Server runs at: **http://localhost:8000**

### 5. Test API

**Swagger UI:** http://localhost:8000/docs
**ReDoc:** http://localhost:8000/redoc

**Test health:**
```bash
curl http://localhost:8000/health
```

**Test transcription:**
```bash
curl -X POST "http://localhost:8000/api/v1/transcribe" \
  -H "Content-Type: multipart/form-data" \
  -F "audio=@test_audio.wav"
```

## API Endpoints

### Health & Status

- `GET /` - API information
- `GET /health` - Health check

### Speech-to-Text

- `POST /api/v1/transcribe` - Transcribe audio file with Whisper API
  - Input: Audio file (wav, mp3, m4a, etc.)
  - Output: `{transcript, language, confidence, duration}`

### Command Parsing

- `POST /api/v1/parse` - Parse natural language to robot command
  - Input: `{command: str, context?: dict}`
  - Output: `{action, parameters, confidence}`

### Real-Time

- `WS /ws` - WebSocket for streaming audio

## Development

### Project Structure

```
backend/
├── app/
│   ├── __init__.py
│   ├── main.py           # FastAPI app & endpoints
│   ├── config.py         # Settings & env vars
│   ├── models/           # Database models (TODO)
│   ├── services/         # Business logic (TODO)
│   └── ros2_client/      # ROS2 integration (TODO)
├── tests/
│   └── __init__.py
├── requirements.txt
├── .env                  # Environment variables (add your keys)
├── .env.example          # Template
└── README.md
```

### Adding New Features

1. **Create service module:**
   ```bash
   touch app/services/whisper_service.py
   ```

2. **Add to main.py:**
   ```python
   from app.services import whisper_service
   ```

3. **Create endpoint:**
   ```python
   @app.post("/api/v1/new-endpoint")
   async def new_endpoint():
       # Implementation
   ```

### Testing

```bash
pytest tests/
```

## Cost Estimates

### Whisper API (OpenAI)
- **$0.006 per minute** of audio
- 100 minutes/month = **$0.60/month**
- 1000 minutes/month = **$6/month**

### Gemini API (Google)
- **Free tier:** 15 requests/minute, 1,500/day
- **Paid:** ~$0.001 per request
- 1000 requests/month = **$1/month** (likely covered by free tier)

**Total:** ~$5-10/month for moderate usage

## Troubleshooting

### "OpenAI API key not found"
- Check `.env` file has `OPENAI_API_KEY=...`
- Restart server after changing `.env`

### "Module not found"
- Ensure virtual environment is activated
- Run `pip install -r requirements.txt`

### "Port 8000 already in use"
- Change port in `.env`: `PORT=8001`
- Or kill existing process: `lsof -ti:8000 | xargs kill`

### ROS2 Integration Issues
- Ensure ROS2 Humble is sourced
- Check `ROS_DOMAIN_ID` matches your ROS2 setup

## Production Deployment

### Docker (Recommended)

```bash
# TODO: Add Dockerfile
docker build -t robot-voice-backend .
docker run -p 8000:8000 robot-voice-backend
```

### Systemd Service

```bash
# TODO: Add systemd service file
sudo systemctl enable robot-voice-backend
sudo systemctl start robot-voice-backend
```

## Week 1 Deliverable

By end of Week 1:
- ✅ FastAPI server running
- ✅ Whisper API transcribing audio
- ✅ Health check endpoint
- ✅ API documentation
- ⏳ WebSocket for streaming
- ⏳ ROS2 integration
- ⏳ Gemini parsing

## Next Steps (Week 2+)

- Week 2: WebSocket + Frontend integration
- Week 3: Database & context memory
- Week 4: ROS2 Nav2 integration
- Week 5: Multi-step commands
- Week 6: Safety validation
- Week 7: 3D visualization

---

**Status:** 🟢 Week 1 Day 1 Complete - Server skeleton ready!
