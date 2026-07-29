import json
from fastapi import FastAPI, Request, HTTPException
from fastapi.responses import StreamingResponse
import httpx

app = FastAPI(title="OpenAI-Compatible Proxy Server")

# Target configurations (Change this to whatever online model/endpoint you want to route to)
TARGET_API_URL = "https://api.regolo.ai/v1/chat/completions"
# Example alternatives:
# "https://api.deepseek.com/v1/chat/completions"
# "https://openrouter.ai/api/v1/chat/completions"

# Fallback API key if the incoming request doesn't provide one
DEFAULT_TARGET_API_KEY = "XXX"


@app.post("/v1/chat/completions")
async def proxy_chat_completions(request: Request):
    # 1. Extract the original payload and headers
    body = await request.json()
    headers = dict(request.headers)

    # 2. Dynamic Model/Routing Rules (Optional)
    # You can forcibly override the model requested by the client
    # body["model"] = "gpt-4o"

    # 3. Resolve the API Key
    # Extract the Bearer token from the incoming client request or use your fallback
    auth_header = headers.get("authorization", f"Bearer {DEFAULT_TARGET_API_KEY}")

    forward_headers = {
        "Authorization": auth_header,
        "Content-Type": "application/json",
    }

    # Setup an asynchronous HTTP client to forward the request
    # Set a generous timeout for LLM responses
    timeout = httpx.Timeout(600.0, connect=10.0)
    client = httpx.AsyncClient(timeout=timeout)

    # 4. Handle Streaming Responses (stream: true)
    if body.get("stream", False):

        async def stream_generator():
            try:
                async with client.stream(
                    "POST", TARGET_API_URL, json=body, headers=forward_headers
                ) as response:
                    if response.status_code != 200:
                        yield f"Error from upstream: {response.status_code}".encode(
                            "utf-8"
                        )
                        return

                    async for chunk in response.aiter_bytes():
                        yield chunk
            except Exception as e:
                yield json.dumps({"error": str(e)}).encode("utf-8")
            finally:
                await client.aclose()

        return StreamingResponse(stream_generator(), media_type="text/event-stream")

    # 5. Handle Standard Responses (stream: false)
    else:
        try:
            async with client as c:
                response = await c.post(
                    TARGET_API_URL, json=body, headers=forward_headers
                )

            if response.status_code != 200:
                raise HTTPException(
                    status_code=response.status_code, detail=response.text
                )

            return response.json()
        except httpx.RequestError as exc:
            raise HTTPException(
                status_code=500,
                detail=f"An error occurred while requesting upstream: {exc}",
            )


# Adding a model list endpoint so client SDKs don't crash when querying available models
@app.get("/v1/models")
async def list_models():
    return {
        "object": "list",
        "data": [
            {"id": "gpt-4o", "object": "model", "owned_by": "openai"},
            {"id": "gpt-4o-mini", "object": "model", "owned_by": "openai"},
        ],
    }


if __name__ == "__main__":
    import uvicorn

    # Run server on port `8000`
    uvicorn.run(app, host="0.0.0.0", port=8000)
