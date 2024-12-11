import os
import io
import uvicorn
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from dotenv import load_dotenv
import streamlit as st
from pydub import AudioSegment
from langchain_community.document_loaders import PyPDFLoader
from langchain.text_splitter import RecursiveCharacterTextSplitter
from langchain_google_genai import GoogleGenerativeAIEmbeddings, ChatGoogleGenerativeAI
from langchain_community.vectorstores import Chroma
from langchain.chains import create_retrieval_chain
from langchain.chains.combine_documents import create_stuff_documents_chain
from langchain_core.prompts import ChatPromptTemplate
from ibm_watson import SpeechToTextV1
from ibm_cloud_sdk_core.authenticators import IAMAuthenticator
from ibm_watson import TextToSpeechV1

load_dotenv()

app = FastAPI(title="RAG with Google Gemini")

class QueryRequest(BaseModel):
    query: str

# --- Seu pipeline RAG (mesmo que antes) ---
pdf_path = "Engineering-workshop-health-and-safety-guidelines-catalog.pdf"
loader = PyPDFLoader(pdf_path)
pages = loader.load()
text_splitter = RecursiveCharacterTextSplitter(chunk_size=1000, chunk_overlap=200, length_function=len)
docs = text_splitter.split_documents(pages)
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
retriever = vectorstore.as_retriever(search_type="mmr", search_kwargs={"k":3, "lambda":0.5})
llm = ChatGoogleGenerativeAI(
    model="gemini-1.5-flash",
    temperature=0.3,
    google_api_key=os.environ.get("GOOGLE_API_KEY")
)
system_prompt = (
    "You are an assistant specialized in industrial safety standards. "
    "Provide concise, clear answers based on the information provided in the context. "
    "If the information is not found in the context, state that the answer is not available. "
    "Limit your response to 3 sentences. "
    "Always respond in English.\n\n"
    "{context}"
)
prompt = ChatPromptTemplate.from_messages(
    [
        ("system", system_prompt),
        ("human", "{input}")
    ]
)
chain = create_stuff_documents_chain(llm, prompt)
rag_chain = create_retrieval_chain(retriever, chain)

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


### STT e TTS ###
def get_speech_to_text():
    try:
        stt_authenticator = IAMAuthenticator(os.environ.get("STT_API_KEY"))
        speech_to_text = SpeechToTextV1(authenticator=stt_authenticator)
        speech_to_text.set_service_url(os.environ.get("STT_SERVICE_URL"))
        return speech_to_text
    except Exception as e:
        st.error(f"Error in setting up Speech to Text service: {e}")
        raise e

speech_to_text = get_speech_to_text()

def transcribe_audio(audio_data):
    # audio_data é bytes contendo o áudio em WAV
    try:
        response = speech_to_text.recognize(
            audio=audio_data,
            content_type='audio/wav',
            model='en-US_BroadbandModel'
        ).get_result()
        results = response.get('results', [])
        if not results:
            st.warning("No transcription results found.")
            return ""
        transcript = results[0]['alternatives'][0]['transcript']
        return transcript.strip()
    except Exception as e:
        st.error(f"Error transcribing audio: {e}")
        return ""

def get_text_to_speech():
    tts_authenticator = IAMAuthenticator(os.environ.get("TTS_API_KEY"))
    text_to_speech = TextToSpeechV1(authenticator=tts_authenticator)
    text_to_speech.set_service_url(os.environ.get("TTS_SERVICE_URL"))
    return text_to_speech

text_to_speech = get_text_to_speech()

def synthesize_speech(text, voice='en-US_AllisonV3Voice', accept='audio/mp3'):
    try:
        audio_content = text_to_speech.synthesize(
            text,
            voice=voice,
            accept=accept
        ).get_result().content
        return audio_content
    except Exception as e:
        st.error(f"Error synthesizing speech: {e}")
        return None

### Interface com Streamlit ###
st.set_page_config(page_title="RAG Chatbot", page_icon="🤖")

if "chat_history" not in st.session_state:
    st.session_state.chat_history = []

st.title("RAG Chatbot - Industrial Safety")

# Mostrar histórico
for i, qa in enumerate(st.session_state.chat_history):
    with st.chat_message("user"):
        st.write(qa['question'])
    with st.chat_message("assistant"):
        st.write(qa['answer'])
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
        audio_data = synthesize_speech(qa['answer'])
        if audio_data:
            st.audio(audio_data, format='audio/mp3')

st.header("Send your query")
tab_text, tab_voice_upload, tab_voice_record = st.tabs(["Text Input", "Voice Upload", "Record Voice (Native)"])

with tab_text:
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
            audio_data = synthesize_speech(result['answer'])
            if audio_data:
                st.audio(audio_data, format='audio/mp3')

with tab_voice_upload:
    st.write("Upload a WAV file with your question (in English):")
    audio_file = st.file_uploader("Upload audio", type=["wav"])
    if audio_file is not None:
        with st.spinner("Transcribing audio..."):
            audio_bytes = audio_file.read()
            transcript = transcribe_audio(audio_bytes)
        if transcript:
            with st.chat_message("user"):
                st.write(transcript)
            with st.spinner("Thinking..."):
                result = rag_chain.invoke({"input": transcript})
                st.session_state.chat_history.append({
                    "question": transcript,
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
                audio_data = synthesize_speech(result['answer'])
                if audio_data:
                    st.audio(audio_data, format='audio/mp3')

with tab_voice_record:
    st.write("Record your question using the microphone (Beta feature):")
    # st.audio_input retorna um UploadedFile se a gravação for realizada
    recorded_file = st.audio_input("Record a voice message", key="native_recorder")
    if recorded_file:
        # O recorded_file é um arquivo WAV em memória
        audio_bytes = recorded_file.getvalue()
        with st.spinner("Transcribing audio..."):
            transcript = transcribe_audio(audio_bytes)
            print(transcript)

        if transcript:
            with st.chat_message("user"):
                st.write(transcript)
            with st.spinner("Thinking..."):
                result = rag_chain.invoke({"input": transcript})
                st.session_state.chat_history.append({
                    "question": transcript,
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
                audio_data = synthesize_speech(result['answer'])
                if audio_data:
                    st.audio(audio_data, format='audio/mp3')

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
