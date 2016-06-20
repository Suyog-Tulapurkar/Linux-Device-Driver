#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/kernel_stat.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/mman.h>
#include <linux/mm.h>
#include <linux/io.h>
#include <linux/ioctl.h>
#include <linux/printk.h>
#include <linux/spinlock.h>
#include <linux/spinlock_types.h>
#include "kyouko3_reg.h"

#define KYOUKO3_MAJOR 500
#define KYOUKO3_MINOR 127
#define KYOUKO_CONTROL_SIZE 65536
#define FIFO_ENTRIES 1024
#define Total_Buffers 8
#define DMA_Size 124*1024

MODULE_LICENSE("Proprietary");
MODULE_AUTHOR("Suyog Tulapurkar");
DEFINE_SPINLOCK(lock);
struct cdev kyouko3_cdev;

struct pci_device_id kyouko3_dev_ids[] = {
    {PCI_DEVICE(0x1234, 0x1113)},
    {0}
};

struct fifoentry{
    u32 command;
    u32 value;
}fifoentry;

struct fifo{
    u64 p_base;
    struct fifoentry *k_base;
    u32 head;
    u32 tail_cache;
};

struct DMA_Buffer{
    unsigned long  p_base;
    unsigned long *k_base;
    unsigned long *u_base;
}DMA[Total_Buffers];

struct u_kyouko_device{
    unsigned int p_control_base;
    unsigned int p_card_ram_base;
    unsigned int *k_control_base;
    unsigned int *k_card_ram_base;
    struct pci_dev *pci_dev;
    struct fifo fifo;
    unsigned int graphics_on;
    unsigned int fill;
    unsigned int drain;
    unsigned int IsDmaUsed;
    unsigned int checkRoot;
}kyouko3;

DECLARE_WAIT_QUEUE_HEAD(dma_snooze);

unsigned int K_READ_REG(unsigned int reg){
    unsigned int value;
   //delay(1);
    rmb();
    value = *(kyouko3.k_control_base + (reg>>2));
    return(value);
}

void K_WRITE_REG(unsigned int reg, unsigned int value){
    udelay(1);
    *(kyouko3.k_control_base + (reg>>2)) = value;
}

int init_fifo(void){
    kyouko3.fifo.k_base = pci_alloc_consistent(kyouko3.pci_dev, 8192u,
                                   &kyouko3.fifo.p_base);
    K_WRITE_REG(FifoStart, kyouko3.fifo.p_base);
    K_WRITE_REG(FifoEnd, (kyouko3.fifo.p_base + 8192u));
    kyouko3.fifo.head = 0;
    kyouko3.fifo.tail_cache = 0;
    printk(KERN_ALERT "%s\n", __FUNCTION__);
    while(K_READ_REG(FifoTail) != 0){
       schedule();
    }
    return 1;
}

void FIFO_WRITE(unsigned int reg, unsigned int value){
    kyouko3.fifo.k_base[kyouko3.fifo.head].command = reg;
    kyouko3.fifo.k_base[kyouko3.fifo.head].value = value;
    kyouko3.fifo.head++;
    if(kyouko3.fifo.head >= FIFO_ENTRIES){
        kyouko3.fifo.head = 0;
    }
}

int kyouko3_open(struct inode *inode, struct file *fd){
    unsigned int ramsize;
    kyouko3.k_control_base = ioremap(kyouko3.p_control_base,
                                 KYOUKO_CONTROL_SIZE);
    ramsize = K_READ_REG(Device_RAM);
    printk(KERN_ALERT "ramsize = %d\n", ramsize);
    kyouko3.k_card_ram_base = ioremap(kyouko3.p_card_ram_base, 1024*1024*ramsize);
    printk(KERN_ALERT "%s\n", __FUNCTION__);
    kyouko3.fill = 0; 
    kyouko3.drain = 0;
    kyouko3.IsDmaUsed = 0;
    kyouko3.checkRoot = current_fsuid().val;
    init_fifo();
    return 0;
}

int kyouko3_release(struct inode *inode, struct file *fd){
    int i;
    if(!kyouko3.IsDmaUsed){
        for(i = 0; i < Total_Buffers; i++){
            vm_munmap(DMA[i].u_base, DMA_Size);
            pci_free_consistent(kyouko3.pci_dev, DMA_Size, DMA[i].k_base, DMA[i].p_base);
        }
    }
    K_WRITE_REG(InterruptSet, 0x00);
    free_irq(kyouko3.pci_dev->irq, &kyouko3);
    pci_disable_msi(kyouko3.pci_dev);
    pci_free_consistent(kyouko3.pci_dev, 8192u, kyouko3.fifo.k_base,
                                 kyouko3.fifo.p_base);
    printk(KERN_ALERT "It is a release message from the kernel\n");

    iounmap(kyouko3.k_control_base);
    iounmap(kyouko3.k_card_ram_base);
    pci_clear_master(kyouko3.pci_dev);
    return 0;
}

int kyouko3_mmap(struct file *fd, struct vm_area_struct *vma){
    int ret;
    /*disable mmap for ordinary users, DMA only*/
    if(!kyouko3.checkRoot){
        if(0 == ((vma->vm_pgoff)<<PAGE_SHIFT)){
        /*control region*/
        ret = io_remap_pfn_range(vma, vma->vm_start,
                    kyouko3.p_control_base>>PAGE_SHIFT,
                         vma->vm_end - vma->vm_start, vma->vm_page_prot);
        }
     }
     else if(0x80000000 == ((vma->vm_pgoff)<<PAGE_SHIFT)){
         /*Frame buffer region*/
      ret = io_remap_pfn_range(vma, vma->vm_start,
                    kyouko3.p_card_ram_base >>PAGE_SHIFT,
                          vma->vm_end - vma->vm_start, vma->vm_page_prot);
     }
     /*Map the DMA buffer*/
     else{
            ret = io_remap_pfn_range(vma, vma->vm_start,
                   DMA[0].p_base>>PAGE_SHIFT,
                          vma->vm_end - vma->vm_start, vma->vm_page_prot);
     }
     return ret;
}

int kyouko3_probe(struct pci_dev *pci_dev, const struct pci_device_id *pci_id){
    printk(KERN_ALERT "%s\n", __FUNCTION__);
    kyouko3.p_control_base = pci_resource_start(pci_dev, 1);
    kyouko3.p_card_ram_base = pci_resource_start(pci_dev, 2);
    kyouko3.pci_dev = pci_dev;
    pci_enable_device(pci_dev);
    pci_set_master(pci_dev);
    return 0;
}

/*Interrupt handler*/
irqreturn_t shandler(int irq, void *dev_id, struct pt_regs *regs){
    unsigned int iflags;
    iflags = K_READ_REG(Status);
    spin_lock_irqsave(&lock, iflags);
    K_WRITE_REG(Status, 0xF);
    if(iflags & 0x02 == 0) {
       spin_unlock_irqrestore(&lock, iflags);
       return IRQ_NONE;
    }
    kyouko3.drain = (kyouko3.drain + 1)%Total_Buffers;
    /*kick the fifo*/
    FIFO_WRITE(Flush, 0x0);
    K_WRITE_REG(FifoHead, kyouko3.fifo.head);
    if(kyouko3.fill == kyouko3.drain - 1){
        wake_up_interruptible(&dma_snooze);
    }
    spin_unlock_irqrestore(&lock, iflags);
    return (IRQ_HANDLED);
}

long kyouko3_ioctl(struct file *fp, unsigned int cmd, unsigned long arg){
    unsigned long ret, count;
    unsigned int i, result, flags;
    float one = 1.0;
    float zero = 0.0;

    unsigned int one_as_int = *(unsigned int*) &one;
    unsigned int zero_as_int = *(unsigned int*) &zero;
    printk(KERN_ALERT "zero_as_int: %d\n", zero_as_int);
    switch(cmd){
        case VMODE:
             printk(KERN_ALERT "graphics on executed\n");
             if(GRAPHICS_ON == arg){
                 /*Set frame to 0*/
                 K_WRITE_REG(Frame_Objects + _FColumns, 1024);
                 K_WRITE_REG(Frame_Objects + _FRows, 768);
                 K_WRITE_REG(Frame_Objects + _FRowPitch, 1024*4);
                 K_WRITE_REG(Frame_Objects + _FFormat, 0xF888);
                 K_WRITE_REG(Frame_Objects + _FAddress, 0);

                 /*Set DAC to 0*/
                 K_WRITE_REG(DAC_Objects + _DWidth, 1024);
                 K_WRITE_REG(DAC_Objects + _DHeight, 768);
                 K_WRITE_REG(DAC_Objects + _DVirtX,    0);
                 K_WRITE_REG(DAC_Objects + _DVirtY,    0);
                 K_WRITE_REG(DAC_Objects + _DFrame,    0);

                 K_WRITE_REG(Acceleration, 0x40000000);

                 K_WRITE_REG(ModeSet, one_as_int);
                 msleep(10);

                 /*Frame buffer registers*/
                 FIFO_WRITE(Clear_Color + 0x000C, zero_as_int);
                 FIFO_WRITE(Clear_Color + 0x0008, zero_as_int);
                 FIFO_WRITE(Clear_Color + 0x0004, zero_as_int);
                 FIFO_WRITE(Clear_Color,     zero_as_int);

                 FIFO_WRITE(Clear_Buffer, 0x030);
                 FIFO_WRITE(Flush, 0x00);
                 kyouko3.graphics_on = 1;
             }
             else {
                 printk(KERN_ALERT "graphics off executed\n");
                 K_WRITE_REG(Acceleration, 0x80000000);
                 K_WRITE_REG(ModeSet, 0);
                 kyouko3.graphics_on = 0;
             }
             break;
        case FIFO_QUEUE:
             printk(KERN_ALERT "FIFO Queue executed\n");
             ret = copy_from_user(&fifoentry, (struct fifoentry *)arg,
                    sizeof(struct fifoentry));
             printk(KERN_ALERT "the value of ret is : %d\n", ret);
             FIFO_WRITE(fifoentry.command, fifoentry.value);
             break;
        case FIFO_FLUSH:
             printk(KERN_ALERT "FIFO FLUSH executed\n");
             K_WRITE_REG(FifoHead, kyouko3.fifo.head);
             while(kyouko3.fifo.tail_cache != kyouko3.fifo.head){
                kyouko3.fifo.tail_cache = K_READ_REG(FifoTail);
                schedule();
               // break;
             }
             printk(KERN_ALERT "FIFO FLUSH executed till end\n");
            break;
        case START_DMA:
             ret = copy_from_user(&count, (unsigned long *)arg, sizeof(unsigned long));
             if(count == 0) return -1;
             spin_lock_irqsave(&lock,flags);
             /*at the start*/
             if(kyouko3.fill == kyouko3.drain){
                 kyouko3.fill = (kyouko3.fill + 1)%Total_Buffers;
                    
                 /*Load BufferA_Address with physical address of DMA buffer*/
                 FIFO_WRITE(BufferA_Address, DMA[kyouko3.fill].p_base);
                 FIFO_WRITE(BufferA_Config, DMA_Size);
                 /*To kick the fifo*/
                 FIFO_WRITE(Flush, 0x0);
                 K_WRITE_REG(FifoHead, kyouko3.fifo.head);
                 spin_unlock_irqrestore(&lock, flags);
                /*copy next available address to user*/
                ret = copy_to_user((unsigned long*)arg, &DMA[i].u_base, sizeof(unsigned long));
                return 0;
               } 
                /*if buffer is  available*/
                kyouko3.fill = (kyouko3.fill + 1)%Total_Buffers;
                if(kyouko3.fill == kyouko3.drain){
                    spin_unlock_irqrestore(&lock, flags);
                    wait_event_interruptible(dma_snooze, kyouko3.fill != kyouko3.drain);
                }
                ret = copy_to_user((unsigned long*)arg, &DMA[i].u_base, sizeof(unsigned long));
                return 0;
                break;
        case BIND_DMA:
             kyouko3.IsDmaUsed = 1;
             /*initialize DMA buffers*/
             printk(KERN_ALERT "%s\n", __FUNCTION__);
             for(i = 0; i < Total_Buffers; i++){
                 DMA[i].k_base = pci_alloc_consistent(kyouko3.pci_dev, DMA_Size, &DMA[i].p_base);
                 DMA[i].u_base = vm_mmap(fp, 0, DMA_Size, PROT_READ|PROT_WRITE, MAP_SHARED, &DMA[i].p_base);
             }
             /*Configure the interrupt handler*/            
             result = pci_enable_msi(kyouko3.pci_dev);
             if(result) return -1;
             result = request_irq(kyouko3.pci_dev->irq, (irq_handler_t)shandler, IRQF_SHARED, "shandler", &kyouko3);
             if(result){
                 pci_disable_msi(kyouko3.pci_dev);
                 return -1;
             }
             K_WRITE_REG(InterruptSet, 0x02);
             /*copy the user base address to the user*/
             ret = copy_to_user((unsigned long*)arg, &DMA[kyouko3.fill].u_base, sizeof(unsigned long));
             break;
        case UNBIND_DMA: 
             if(kyouko3.IsDmaUsed){
                 for(i = 0; i < Total_Buffers; i++){
                     vm_munmap(DMA[i].u_base, DMA_Size);
                     pci_free_consistent(kyouko3.pci_dev, DMA_Size, DMA[i].k_base, DMA[i].p_base);
                 }
             }
             break;
        default:
            break;
      }
      return 0;
}

void kyouko3_remove(struct pci_dev *pci_dev){
    pci_disable_device(pci_dev);
    printk(KERN_ALERT "in the remove\n");
}

struct file_operations kyouko3_fops = {
    .open = kyouko3_open,
    .release = kyouko3_release,
    .unlocked_ioctl = kyouko3_ioctl,
    .mmap   = kyouko3_mmap,
    .owner  = THIS_MODULE
};

struct pci_driver kyouko3_pci_drv = {
   .name      = "my_kyouko3",
   .id_table  = kyouko3_dev_ids,
   .probe     = kyouko3_probe,
   .remove    = kyouko3_remove,
};

static int __init my_init_function(void){
    //struct cdev mystruct;
    cdev_init(&kyouko3_cdev, &kyouko3_fops);
    //kyouko3_cdev.owner = THIS_MODULE;
    cdev_add(&kyouko3_cdev, MKDEV(KYOUKO3_MAJOR, KYOUKO3_MINOR),1);
    pci_register_driver(&kyouko3_pci_drv);
    printk(KERN_ALERT "init function executed\n");
    return 0;
}

static void __exit my_exit_function(void){
    pci_unregister_driver(&kyouko3_pci_drv);
    cdev_del(&kyouko3_cdev);
    printk(KERN_ALERT "exit function executed\n");
}

module_init(my_init_function);
module_exit(my_exit_function);
