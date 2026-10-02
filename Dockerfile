FROM python:3.12-slim AS builder

WORKDIR /opt/build
COPY requirements.txt /opt/build/requirements.txt
COPY src /opt/build/src
RUN pip install --no-cache-dir -r /opt/build/requirements.txt \
    && python -m compileall -q /opt/build/src

FROM python:3.12-slim

RUN useradd --uid 10001 --no-create-home --shell /usr/sbin/nologin engine
WORKDIR /app
COPY --from=builder /opt/build/src /app/src
USER 10001
ENTRYPOINT ["python", "src/main.py"]
