FROM docker.io/python:3.11-slim
RUN pip3 install --no-cache-dir paho-mqtt
WORKDIR /app
COPY hub.py dashboard.html /app/
CMD ["python3", "hub.py"]
