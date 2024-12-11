import streamlit as st
from chatbot import initialize_rag_pipeline
from stt_tts import transcribe_audio, synthesize_speech

# pipeline do RAG
rag_chain = initialize_rag_pipeline()

st.set_page_config(page_title="RAG Chatbot", page_icon="🤖")

# Históricos de conversa e estados
if "chat_history" not in st.session_state:
    st.session_state.chat_history = []
if "recorded_audio_transcript" not in st.session_state:
    st.session_state.recorded_audio_transcript = None
if "audio_processed" not in st.session_state:
    st.session_state.audio_processed = True  # vamo marcar que o audio já foi processado

st.title("Chatbot - Engineering Workshop Health & Safety Guidelines Catalog")

# histórico de msg para ficar tipo o chat gpt
for qa in st.session_state.chat_history:
    with st.chat_message("user"):
        st.write(qa["question"])
    with st.chat_message("assistant"):
        st.write(qa["answer"])
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
        audio_data = synthesize_speech(qa["answer"])
        if audio_data:
            st.audio(audio_data, format="audio/mp3")

# gravação de áudio
st.write("Record your question (English):")
recorded_audio = st.audio_input("Record a voice message", key="audio_record")
if recorded_audio and st.session_state.audio_processed:
    with st.spinner("Transcribing audio..."):
        audio_bytes = recorded_audio.getvalue()
        transcript = transcribe_audio(audio_bytes)
    if transcript:
        st.session_state.recorded_audio_transcript = transcript
        st.session_state.audio_processed = False  # Marcar como não processado
        with st.chat_message("user"):
            st.write(transcript)

# Processar a transcrição do áudio, se disponível e não processada
if st.session_state.recorded_audio_transcript and not st.session_state.audio_processed:
    with st.spinner("Thinking..."):
        result = rag_chain.invoke({"input": st.session_state.recorded_audio_transcript})
    st.session_state.chat_history.append({
        "question": st.session_state.recorded_audio_transcript,
        "answer": result["answer"],
        "context": result["context"]
    })
    st.session_state.recorded_audio_transcript = None
    st.session_state.audio_processed = True  # Marcar como processado
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
        audio_data = synthesize_speech(result["answer"])
        if audio_data:
            st.audio(audio_data, format="audio/mp3")

# Campo para entrada de texto
user_input = st.chat_input("Type your question or record audio...")
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
        audio_data = synthesize_speech(result["answer"])
        if audio_data:
            st.audio(audio_data, format="audio/mp3")
