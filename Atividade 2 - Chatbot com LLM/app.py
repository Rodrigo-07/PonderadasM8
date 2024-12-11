from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from chatbot import initialize_rag_pipeline

app = FastAPI(title="RAG with Google Gemini")

class QueryRequest(BaseModel):
    query: str

rag_chain = initialize_rag_pipeline()

@app.post("/chat")
def chat(query_request: QueryRequest):
    query = query_request.query.strip()
    if not query:
        raise HTTPException(status_code=400, detail="The query cannot be empty.")
    try:
        answer = rag_chain.invoke({"input": query})
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
    return {"answer": answer}
