FROM python:3.12-slim
WORKDIR /app
COPY requirements.txt .
# Install torch CPU-only first (avoids downloading the 2GB CUDA wheel)
RUN pip install --no-cache-dir --timeout 120 \
    torch --extra-index-url https://download.pytorch.org/whl/cpu
RUN pip install --no-cache-dir --timeout 120 -r requirements.txt \
    --extra-index-url https://download.pytorch.org/whl/cpu
COPY . .
EXPOSE 8000
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
