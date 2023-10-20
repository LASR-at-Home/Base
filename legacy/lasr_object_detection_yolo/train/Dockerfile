FROM python:3 as builder

COPY . .
ENV PATH $PATH:/scripts

ENV USE_GPU 0
ENV USE_OPENCV 0
ENV USE_OPENMP 0
ENV USE_CUDNN 0

RUN mkdir datasets config weights
RUN build_darknet.sh

ENTRYPOINT ["train.sh"]