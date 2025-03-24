# python_subscriber/Dockerfile
FROM python:3.9-slim

WORKDIR /app

# Install required packages
COPY requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt

# Copy the subscriber script
COPY subscriber.py ./

CMD ["python", "subscriber.py"]
