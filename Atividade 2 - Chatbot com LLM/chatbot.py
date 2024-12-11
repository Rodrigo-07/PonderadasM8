import os
from langchain_community.document_loaders import PyPDFLoader
from langchain.text_splitter import RecursiveCharacterTextSplitter
from langchain_google_genai import GoogleGenerativeAIEmbeddings, ChatGoogleGenerativeAI
from langchain_community.vectorstores import Chroma
from langchain.chains import create_retrieval_chain
from langchain.chains.combine_documents import create_stuff_documents_chain
from langchain_core.prompts import ChatPromptTemplate
from utils.pdf_loader import load_and_split_pdf
from dotenv import load_dotenv

load_dotenv()

def initialize_rag_pipeline():
    # Carregar e dividir documentos
    docs = load_and_split_pdf("Engineering-workshop-health-and-safety-guidelines-catalog.pdf")
    
    # numero de chunks
    print(f"Chunks: {len(docs)}")
    
    # for doc in docs:
    #     print(f"Page: {doc.metadata.get('page', 'unknown')}, Source: {doc.metadata.get('source', 'unknown')}")
    #     print(f"Excerpt:\n{doc.page_content[:500]}...")

    # Criar embeddings e vetorstore
    # embeding -> vector representation of the text
    embeddings = GoogleGenerativeAIEmbeddings(
        model="models/embedding-001",
        google_api_key=os.getenv("GOOGLE_API_KEY")
    )
    
    # Vamo usar o Chroma para armazenar os embeddings
    vectorstore = Chroma.from_documents(
        docs,
        embedding=embeddings,
        collection_name="my_documents",
        persist_directory="./chromadb"
    )
    
    # Retriever serve para recuperar os documentos mais relevantes, contexto e depois passar para o LLM gerar a resposta
    retriever = vectorstore.as_retriever(search_type="mmr", search_kwargs={"k": 3, "lambda": 0.5})

    # Config LLM e prompts
    llm = ChatGoogleGenerativeAI(
        model="gemini-1.5-flash",
        temperature=0.3,
        google_api_key=os.getenv("GOOGLE_API_KEY")
    )
    system_prompt = (
        "You are an assistant specialized in industrial safety standards. "
        "Provide concise, clear answers based on the information provided in the context. "
        "If the information is not found in the context, state that the answer is not available. "
        "Limit your response to 3 sentences. If really necessary, you can provide up to 5 sentences. "
        "If you need more information, ask the user for clarification. "
        "Always respond in English.\n\n"
        "{context}"
    )
    prompt = ChatPromptTemplate.from_messages(
        [("system", system_prompt), ("human", "{input}")]
    )
    
    # Rag chain é o pipeline completo de processamento, desde a recuperação dos documentos até a geração da resposta
    chain = create_stuff_documents_chain(llm, prompt)
    
    # Retorna o pipeline completo então recebe o retriever e o chain.
    # pergunta -> embeeding -> procura o vetores mais proximos -> retrive os documentos -> manda para o LLM -> gera a resposta
    return create_retrieval_chain(retriever, chain)
