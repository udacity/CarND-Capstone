FROM bydavy/carnd-capstone

# Install python dependencies
COPY requirements.txt /tmp/requirements.txt
RUN pip install -r /tmp/requirements.txt