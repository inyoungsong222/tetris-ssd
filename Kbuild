obj-m   := nvmev.o
nvmev-objs := main.o pci.o admin.o io.o dma.o

TENANT_N ?= 4
ccflags-y += -DTENANT_N=$(TENANT_N)

nvmev-y += ssd.o conv_ftl.o pqueue/pqueue.o channel_model.o cache_model.o

ccflags-y += -DGC_THRES_LINES=2

