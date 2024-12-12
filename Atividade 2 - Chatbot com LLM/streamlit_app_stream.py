import os
import streamlit as st
import google.generativeai as genai
from dotenv import load_dotenv
from langchain_community.document_loaders import PyPDFLoader
from langchain.text_splitter import RecursiveCharacterTextSplitter
from langchain_google_genai import GoogleGenerativeAIEmbeddings
from langchain_community.vectorstores import Chroma
from stt_tts import transcribe_audio, synthesize_speech
from utils.pdf_loader import load_and_split_pdf
from langchain_core.prompts import ChatPromptTemplate
from langchain_core.messages import AIMessage, HumanMessage
from langchain_google_genai import ChatGoogleGenerativeAI
from langchain_core.output_parsers import StrOutputParser

load_dotenv()

# Key do Gemini
genai.configure(api_key=os.getenv("GOOGLE_API_KEY"))

# config inicial do LLM
def get_llm_instance():
    global llm
    if "llm" not in st.session_state:
        st.session_state.llm = ChatGoogleGenerativeAI(
            model="gemini-1.5-flash",
            stream=True,
        )
    return st.session_state.llm

# Vamo gerar as resposta com  base no histórico de conversa, para manter o contexto
def get_response(user_query, conversation_history):
    # juntar o histórico de conversa
    formatted_history = "\n".join(
        f"User: {msg.content}" if isinstance(msg, HumanMessage) else f"Assistant: {msg.content}"
        for msg in conversation_history
    )
    

    # Template do prompt
    prompt_template = """
    You are an assistant specialized in industrial safety standards. 
    Provide concise, clear answers based on the information provided in the context.
    If the information is not found in the context, state that the answer is not available.
    Limit your response to 5 sentences.
    If you need more information, ask the user for clarification.
    Always respond in English.
    
    Answer the following question considering the history of the conversation:
    
    Chat history:
    {conversation_history}
    
    User question:
    {user_query}
    """
    
    # Usar o from_template para criar o prompt, pq ele já formato o prompt
    prompt = ChatPromptTemplate.from_template(template=prompt_template)
    
    # Gerando uma instancia do lllm
    llm = get_llm_instance()
    
    # cadeia de processamento -> isso vai gerar tipo um pipeline de processamento que tem o llm, prompt e um parser de output (que serve para formatar a resposta)
    expression_language_chain = prompt | llm | StrOutputParser()
    
    # streaming
    return expression_language_chain.stream(
        {
            "conversation_history": formatted_history,
            "user_query": user_query
        }
    )

# Logica de embedding

docs = load_and_split_pdf("Engineering-workshop-health-and-safety-guidelines-catalog.pdf")

# embeddings e vectorstore
embeddings = GoogleGenerativeAIEmbeddings(
    model="models/embedding-001",
    google_api_key=os.getenv("GOOGLE_API_KEY")
)

vectorstore = Chroma.from_documents(
    docs,
    embedding=embeddings,
    collection_name="my_documents",
    persist_directory="./chromadb"
)

retriever = vectorstore.as_retriever(search_type="mmr", search_kwargs={"k": 3, "lambda": 0.5})

# Inicializar o Streamlit
st.set_page_config(page_title="RAG Chatbot", page_icon="🤖")

# Iniciliazar o histórico de mensagens
if "messages" not in st.session_state:
    st.session_state.messages = [
        AIMessage(content="Hello, I am a bot. How can I help you?")
    ]

st.title("Chatbot - Engineering Workshop Health & Safety Guidelines Catalog")

# histórico de mensagens
for message in st.session_state.messages:
    if isinstance(message, AIMessage):
        with st.chat_message("assistant"):
            st.write(message.content)
    elif isinstance(message, HumanMessage):
        with st.chat_message("user"):
            st.write(message.content)

# Pegar o texto
typed_message = st.chat_input("Type your question or record audio...")

if typed_message:
    
    # Add a mensagem do usuario no histórico
    st.session_state.messages.append(HumanMessage(content=typed_message))
    with st.chat_message("user"):
        st.write(typed_message)

    with st.chat_message("assistant"):
        
        # Stream da mensage
        response = st.write_stream(get_response(typed_message, st.session_state.messages))
        
        # Adicionar a resposta no histórico
        st.session_state.messages.append(AIMessage(content=response))
        
        audio_data = synthesize_speech(response)
        
        if audio_data:
            st.audio(audio_data, format="audio/mp3")

# Áudio
st.write("Record your question (English):")
recorded_audio = st.audio_input("Record a voice message", key="audio_record")
if recorded_audio:
    with st.spinner("Transcribing audio..."):
        audio_bytes = recorded_audio.getvalue()
        transcript = transcribe_audio(audio_bytes)
    if transcript:
        st.session_state.messages.append(HumanMessage(content=transcript))
        with st.chat_message("user"):
            st.write(transcript)

        with st.chat_message("assistant"):
            response = st.write_stream(get_response(transcript, st.session_state.messages))
            st.session_state.messages.append(AIMessage(content=response))
