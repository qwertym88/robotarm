#include "serial.h"

uint8_t message[400]; // 接收字符串环形缓冲区
uint8_t offset;       // 接收字符串缓冲区的下标及大小
uint8_t mesg;         // 用于中断时，接收单个字符
uint8_t RX_Flag;      // 发生中断的标志
uint8_t GAIN_F;       // 接受完整语句的标志
uint8_t seekp = 0;

/**
 * @brief 重写串口中断
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (offset == 400)
        offset = 0;
    if (mesg != '\r' && mesg != '\n' && mesg != 255) // 忽略这些字符,反正本来也没用
        message[offset++] = mesg;
    RX_Flag = 1;
    if (mesg == ';') // 读到;说明已到达完整语句结尾
        GAIN_F = 1;
    HAL_UART_Receive_IT(huart, (uint8_t *)&mesg, 1);
}
/**
 * @brief 从缓冲区读取字符串
 * @param buf 接收的字符串
 * @retval 读取的字符串长度
 */
int readStr(uint8_t *buf)
{
    uint8_t tail = seekp;
    uint8_t flag = 0;
    while (message[tail++] != ';')
    {
        if (tail == 400)
        {
            tail = 0;
            if (flag == 1) // 都没有;则直接返回空
                buf = NULL;
            flag = 1;
        }
    }
    tail--; // 形如123;的返回123'\0'，去掉;
    if (flag == 0)
    {
        memcpy(buf, message + seekp, tail - seekp);
        buf[tail - seekp] = '\0';
    }
    else
    {
        memcpy(buf, message + seekp, 400 - seekp);
        memcpy(buf + (400 - seekp), message, tail);
        buf[tail] = '\0';
    }
    if (tail + 1 == offset) // 若已经读完缓冲区,清除GAIN_F标志
        GAIN_F = 0;
    int len = tail - seekp + 1;
    seekp = tail + 1;
    return len;
}

/**
 * @brief 从缓冲区读取字符串
 * @retval 是否已至少读入一条完整语句
 */
uint8_t UART_available(void)
{
    return GAIN_F;
}

/**
 * @brief 由于奇怪的原因，用makefile生成时sprintf无法处理小数，keil没有这个问题。参照教程加了-u _printf_float，但仍旧无效。故采用这种方法
 */
void print_float(double value)
{
    int z, tmp1, tmp2, tmp3, tmp4, tmp5, tmp6;
    z = (int)value;       // 整数部分+符号
    double f = value - z; // 小数部分
    f = f < 0 ? -f : f;   // abs
    tmp1 = (int)(f * 10) % 10;
    tmp2 = (int)(f * 100) % 10;
    tmp3 = (int)(f * 1000) % 10;
    tmp4 = (int)(f * 10000) % 10;
    tmp5 = (int)(f * 100000) % 10;
    tmp6 = (int)(f * 1000000) % 10;
    printf("%d.%d%d%d%d%d%d ", z, tmp1, tmp2, tmp3, tmp4, tmp5, tmp6);
}

/**
 * @brief 输出详细日志，由于printf无法处理小数，故用此代替
 */
void print_debug(char *str, double a, double b, double c, char *end)
{
    if (CURRENT_LEVEL <= DEBUG_LEVEL)
    {
        USART_OUT;
        printf("[debug] %s", str);
        print_float(a);
        print_float(b);
        print_float(c);
        printf("%s", end);
        USART_IN;
    }
}

/**
 * @brief 输出日志，由于printf无法处理小数，故用此代替
 */
void print_info(char *str, double a, double b, double c, char *end)
{
    if (CURRENT_LEVEL <= INFO_LEVEL)
    {
        USART_OUT;
        printf("[info] %s", str);
        print_float(a);
        print_float(b);
        print_float(c);
        printf("%s", end);
        USART_IN;
    }
}

/**
 * @brief 返回命令结果，由于printf无法处理小数，故用此代替
 */
void print_release(char *str, double a, double b, double c, char *end)
{
    if (CURRENT_LEVEL == RELEASE_LEVEL)
    {
        m_printf("%s", str);
        print_float(a);
        print_float(b);
        print_float(c);
        m_printf("%s", end);
    }
}

// int fputc(int ch, FILE *f)
// {
//   HAL_UART_Transmit(&huart2,(uint8_t *)&ch,1,0xFFFF);//阻塞方式打印,串口1
//   return ch;
// }
