import os
import uvicorn
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from dotenv import load_dotenv
import streamlit as st
from langchain_community.document_loaders import PyPDFLoader
from langchain.text_splitter import RecursiveCharacterTextSplitter
from langchain_google_genai import GoogleGenerativeAIEmbeddings, ChatGoogleGenerativeAI
from langchain_community.vectorstores import Chroma
from langchain.chains import create_retrieval_chain
from langchain.chains.combine_documents import create_stuff_documents_chain
from langchain_core.prompts import ChatPromptTemplate

load_dotenv()

app = FastAPI(title="RAG with Google Gemini")

class QueryRequest(BaseModel):
    query: str

# PDF
pdf_path = "Engineering-workshop-health-and-safety-guidelines-catalog.pdf"

# Carregar o PDF
loader = PyPDFLoader(pdf_path)
pages = loader.load()

# Divide o texto em chunks
text_splitter = RecursiveCharacterTextSplitter(chunk_size=1000, chunk_overlap=200, length_function=len)
docs = text_splitter.split_documents(pages)

# Cria embeddings
embeddings = GoogleGenerativeAIEmbeddings(
    model="models/embedding-001",
    google_api_key=os.getenv("GOOGLE_API_KEY")
)

# Cria o vetor store, no Chroma db, a partir dos documentos
vectorstore = Chroma.from_documents(
    docs, 
    embedding=embeddings, 
    collection_name="my_documents", 
    persist_directory="./chromadb"
)

# Cria um retriever -> que é um modelo de recuperação de informações melhor que um simples modelo de similaridade
# retriever = vectorstore.as_retriever(search_type="similarity", search_kwargs={"k":3})
retriever = vectorstore.as_retriever(search_type="mmr", search_kwargs={"k":3, "lambda":0.5})

# modelo LLM com Gemini
llm = ChatGoogleGenerativeAI(
    model="gemini-1.5-flash",
    temperature=0.3,
    google_api_key=os.environ.get("GOOGLE_API_KEY")
)

#  system_prompt que será usado como base para o chat
system_prompt = (
    "You are an assistant specialized in industrial safety standards. "
    "Provide concise, clear answers based on the information provided in the context. "
    "If the information is not found in the context, state that the answer is not available. "
    "Limit your response to 3 sentences. "
    "Always respond in English.\n\n"
    "{context}"
)

# Cria o prompt template para o chat, com uma mensagem de sistema e uma mensagem do usuário que é o human
prompt = ChatPromptTemplate.from_messages(
    [
        ("system", system_prompt),
        ("human", "{input}")
    ]
)

# Cria a chain "stuff" para combinar os documentos recuperados
chain = create_stuff_documents_chain(llm, prompt)

# Cria a chain de RAG
rag_chain = create_retrieval_chain(retriever, chain)

# Endpoint para o chat
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


# Interface com Streamlit


st.set_page_config(page_title="RAG Chatbot", page_icon="🤖")

if "chat_history" not in st.session_state:
    st.session_state.chat_history = []

st.title("RAG Chatbot - Industrial Safety")

# Exibe o histórico de mensagens antes da nova entrada
for i, qa in enumerate(st.session_state.chat_history):
    # Mensagem do usuário
    with st.chat_message("user"):
        st.write(qa['question'])
    # Mensagem do assistente
    with st.chat_message("assistant"):
        st.write(qa['answer'])
        # Expander para referências
        with st.expander("Show References"):
            refs = qa["context"]
            if refs:
                st.write("References used:")
                for doc in refs:
                    page = doc.metadata.get("page", "unknown")
                    source = doc.metadata.get("source", "unknown")
                    st.write(f"- Page: {page}, Source: {source}")
                    st.write(f"Excerpt:\n{doc.page_content[:500]}...")
            else:
                st.write("No references available.")

# Campo de entrada estilo chat
user_input = st.chat_input("Ask your question...")
if user_input:
    with st.chat_message("user"):
        st.write(user_input)
    with st.spinner("Thinking..."):
        result = rag_chain.invoke({"input": user_input})
        st.session_state.chat_history.append({
            "question": user_input,
            "answer": result["answer"],
            "context": result["context"]
        })
        
    with st.chat_message("assistant"):
        st.write(result["answer"])
        with st.expander("Show References"):
            refs = result["context"]
            if refs:
                st.write("References used:")
                for doc in refs:
                    page = doc.metadata.get("page", "unknown")
                    source = doc.metadata.get("source", "unknown")
                    st.write(f"- Page: {page}, Source: {source}")
                    st.write(f"Excerpt:\n{doc.page_content[:500]}...")
            else:
                st.write("No references available.")

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
