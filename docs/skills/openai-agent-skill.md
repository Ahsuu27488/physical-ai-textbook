# OpenAI Agents SDK Skill

## Installation

### Basic installation:
```bash
pip install openai-agents
```

### With additional features:
```bash
# With voice support
pip install 'openai-agents[voice]'

# With Redis session support
pip install 'openai-agents[redis]'

# With SQLAlchemy support
pip install 'openai-agents[sqlalchemy]'

# With encryption support
pip install 'openai-agents[encrypt]'

# With visualization support
pip install 'openai-agents[viz]'

# With LiteLLM support for multiple AI providers
pip install 'openai-agents[litellm]'
```

### Using uv (recommended):
```bash
uv init
uv add openai-agents
```

## Initialize and Run an Agent

### Basic Agent:
```python
from agents import Agent, Runner

# Create a simple agent with instructions
agent = Agent(
    name="Assistant",
    instructions="You are a helpful assistant that responds in haikus."
)

# Synchronous execution
result = Runner.run_sync(agent, "Tell me about recursion in programming.")
print(result.final_output)

# Asynchronous execution
import asyncio

async def main():
    result = await Runner.run(agent, "Explain machine learning.")
    print(result.final_output)

    # Access conversation items
    for item in result.new_items:
        print(item)

asyncio.run(main())
```

### Agent with Tools:
```python
from agents import Agent, Runner, function_tool

@function_tool
def get_weather(city: str):
    """Get the weather for a city."""
    print(f"[debug] getting weather for {city}")
    return f"The weather in {city} is sunny."

agent = Agent(
    name="Assistant",
    instructions="You only respond in haikus.",
    tools=[get_weather],
)

async def main():
    result = await Runner.run(agent, "What's the weather in Tokyo?")
    print(result.final_output)

asyncio.run(main())
```

### Agent with Session Memory (Persistent):
```python
from agents import Agent, Runner, SQLiteSession

# Create agent
agent = Agent(
    name="Assistant",
    instructions="Reply very concisely.",
)

# Create a session instance
session = SQLiteSession("conversation_123")

# First turn
result = await Runner.run(
    agent,
    "What city is the Golden Gate Bridge in?",
    session=session
)
print(result.final_output)  # "San Francisco"

# Second turn - agent automatically remembers previous context
result = await Runner.run(
    agent,
    "What state is it in?",
    session=session
)
print(result.final_output)  # "California"
```

### Agent with Non-OpenAI Models (via LiteLLM):
```python
from agents import Agent

# Using Claude
claude_agent = Agent(
    model="litellm/anthropic/claude-3-5-sonnet-20240620",
    name="Claude Assistant",
    instructions="You are a helpful assistant."
)

# Using Gemini
gemini_agent = Agent(
    model="litellm/gemini/gemini-2.5-flash-preview-04-17",
    name="Gemini Assistant",
    instructions="You are a helpful assistant."
)
```

### Realtime Agent (Voice):
```python
import asyncio
from agents.realtime import RealtimeAgent, RealtimeRunner

async def main():
    # Create the agent
    agent = RealtimeAgent(
        name="Assistant",
        instructions="You are a helpful voice assistant. Keep responses brief and conversational.",
    )

    # Set up the runner with configuration
    runner = RealtimeRunner(
        starting_agent=agent,
        config={
            "model_settings": {
                "model_name": "gpt-realtime",
                "voice": "ash",
                "modalities": ["audio"],
                "input_audio_format": "pcm16",
                "output_audio_format": "pcm16",
                "input_audio_transcription": {"model": "gpt-4o-mini-transcribe"},
                "turn_detection": {"type": "semantic_vad", "interrupt_response": True},
            }
        },
    )

    # Start the session
    session = await runner.run()

    async with session:
        print("Session started! The agent will stream audio responses in real-time.")
        # Process events
        async for event in session:
            if event.type == "agent_start":
                print(f"Agent started: {event.agent.name}")
            elif event.type == "agent_end":
                print(f"Agent ended: {event.agent.name}")
            elif event.type == "tool_start":
                print(f"Tool started: {event.tool.name}")
            elif event.type == "tool_end":
                print(f"Tool ended: {event.tool.name}; output: {event.output}")
            elif event.type == "error":
                print(f"Error: {event.error}")

if __name__ == "__main__":
    asyncio.run(main())
```