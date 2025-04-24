import os

import google.generativeai as genai

from shared_queue import log_queue_gemini

# --- Configuration ---
# IMPORTANT: Set your API key as an environment variable
# export GOOGLE_API_KEY="YOUR_API_KEY"
# Or replace os.getenv("GOOGLE_API_KEY") with your key directly (less secure)
API_KEY = "AIzaSyAdf5Mg-42Ccd6lON8S3Rr2kK5okWHV53Q"  # 替換為你的實際金鑰
if not API_KEY:
    log_queue_gemini.put("[GeminiClient] Error: GOOGLE_API_KEY environment variable not set.")
    # You might want to raise an exception or handle this more gracefully
    # For now, we'll allow it to proceed but API calls will fail.

DEFAULT_MODEL = "gemini-1.5-flash" # Or choose another suitable model

class GeminiClient:
    """Handles interactions with the Google Gemini API."""
    def __init__(self, api_key=API_KEY):
        self.api_key = api_key
        self.model = None
        self._configure()

    def _configure(self):
        """Configures the Gemini API client."""
        if not self.api_key:
            log_queue_gemini.put("[GeminiClient] Configuration skipped: API key is missing.")
            return
        try:
            genai.configure(api_key=self.api_key)
            log_queue_gemini.put("[GeminiClient] Gemini API configured successfully.")
        except Exception as e:
            log_queue_gemini.put(f"[GeminiClient] Error configuring Gemini API: {e}")

    def set_model(self, model_name=DEFAULT_MODEL):
        """Sets the generative model to use."""
        if not self.api_key:
            log_queue_gemini.put("[GeminiClient] Cannot set model: API key is missing.")
            return False
        try:
            self.model = genai.GenerativeModel(model_name)
            log_queue_gemini.put(f"[GeminiClient] Model set to: {model_name}")
            return True
        except Exception as e:
            log_queue_gemini.put(f"[GeminiClient] Error setting model '{model_name}': {e}")
            self.model = None
            return False

    def generate_response(self, prompt):
        """Sends a prompt to the configured Gemini model and returns the response."""
        if not self.model:
            log_queue_gemini.put("[GeminiClient] Cannot generate response: Model not set or failed to initialize.")
            return "Error: Model not available."
        try:
            log_queue_gemini.put(f"[GeminiClient] Sending prompt to model...")
            response = self.model.generate_content(prompt)
            log_queue_gemini.put(f"[GeminiClient] Received response from model.")
            # Basic error handling for response structure
            if response and hasattr(response, 'text'):
                 return response.text
            elif response and response.prompt_feedback:
                 # Handle cases where content might be blocked
                 log_queue_gemini.put(f"[GeminiClient] Prompt feedback: {response.prompt_feedback}")
                 return f"Error: Could not generate response. Feedback: {response.prompt_feedback}"
            else:
                 log_queue_gemini.put(f"[GeminiClient] Unexpected response format: {response}")
                 return "Error: Unexpected response format from API."

        except Exception as e:
            log_queue_gemini.put(f"[GeminiClient] Error generating response: {e}")
            return f"Error: API call failed. {e}"

    @staticmethod
    def list_available_models():
        """Lists available Gemini models (requires API key)."""
        if not API_KEY:
            log_queue_gemini.put("[GeminiClient] Cannot list models: API key is missing.")
            return ["gemini-1.5-flash", "gemini-pro"] # Provide defaults if key missing
        try:
            genai.configure(api_key=API_KEY) # Ensure configured
            models = [m.name for m in genai.list_models() if 'generateContent' in m.supported_generation_methods]
            # Filter for relevant models if needed, e.g., starts with 'models/gemini'
            models = [m.split('/')[-1] for m in models if m.startswith('models/gemini')]
            log_queue_gemini.put(f"[GeminiClient] Available models: {models}")
            return models if models else ["gemini-1.5-flash", "gemini-pro"] # Fallback
        except Exception as e:
            log_queue_gemini.put(f"[GeminiClient] Error listing models: {e}")
            return ["gemini-1.5-flash", "gemini-pro"] # Fallback on error
