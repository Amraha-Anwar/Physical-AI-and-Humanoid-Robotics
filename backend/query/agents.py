import os
from typing import List, Callable, Awaitable
from dotenv import load_dotenv
from openai import AsyncOpenAI

from agents import (
    Agent,
    OpenAIChatCompletionsModel,
    Runner,
    function_tool,
)

# --------------------
# Environment
# --------------------
load_dotenv()

# --------------------
# Tool Factory
# --------------------
def make_retrieval_tool(
    retrieval_fn: Callable[[str], Awaitable[List[str]]]
):
    @function_tool
    async def retrieve_context(query: str) -> str:
        """
        Retrieve relevant context from the knowledge base.
        """
        chunks = await retrieval_fn(query)

        if not chunks:
            return "No relevant context found in the knowledge base."

        return "\n\n".join(chunks)

    return retrieve_context

# --------------------
# Factory for Agents
# --------------------
def get_agents(client: AsyncOpenAI):
    model = OpenAIChatCompletionsModel(
        model="gpt-4o-mini",
        openai_client=client,
    )

    # T106.1: Final Refined System Instructions
    reasoning_agent = Agent(
        name="Reasoning Agent",
        model=model,
        instructions="""
You are the 'Physical AI & Humanoid Robotics' expert assistant.

Your priority is to answer using the retrieved context. 
- When you find information in the context, always use bullet points for comparisons or lists.
- Be confident and detailed.
- Mention that you are sourcing this from the project documentation.

If no context is found, politely state that the specific detail isn't in the current documentation version.

Style:
- Clear
- Educational
- Beginner-friendly
""",
    )
    return model, reasoning_agent

# --------------------
# Orchestrator Agent Factory
# --------------------
def make_orchestrator_agent(retrieval_fn, model, reasoning_agent):
    retrieve_context = make_retrieval_tool(retrieval_fn)

    return Agent(
        name="RAG Orchestrator",
        model=model,
        instructions="""
You are a RAG Orchestrator.

First, classify the user's message:
- If it is a casual greeting or small talk (e.g. "hi", "hello", "hey", "good morning",
  "thanks", "how are you", "who are you"), or any other non-technical message that
  does not ask about Physical AI, humanoid robotics, or the book's content, do NOT
  call retrieve_context. Respond directly with a short, friendly message such as:
  "Hi! I'm your Physical AI & Humanoid Robotics assistant. Ask me anything about the book."
  Adapt the wording to fit the greeting, but keep it brief and welcoming.

- Otherwise, treat the message as a substantive question and follow the RAG steps below.

RAG Steps (only for substantive questions):
1. Call the retrieve_context tool.
2. Pass the retrieved context forward.
3. Hand off to the Reasoning Agent.

Rules:
- For substantive questions: never answer directly; always retrieve context first.
- For greetings/small talk: answer directly without retrieval.
""",
        tools=[retrieve_context],
        handoffs=[reasoning_agent],
    )

# --------------------
# Execution Entry Point
# --------------------
async def run_rag_pipeline(
    user_query: str,
    retrieval_fn: Callable[[str], Awaitable[List[str]]],
) -> str:
    # Instantiate client here (Lazy Loading)
    client = AsyncOpenAI(
        api_key=os.getenv("OPENAI_API_KEY"),
    )
    
    model, reasoning_agent = get_agents(client)
    orchestrator = make_orchestrator_agent(retrieval_fn, model, reasoning_agent)

    # T091.3: Verification Log (Optional/Temporary)
    # print(f"DEBUG: Reasoning Agent Instructions:\n{reasoning_agent.instructions}")

    result = await Runner.run(
        orchestrator,
        input=user_query,
    )

    return result.final_output
