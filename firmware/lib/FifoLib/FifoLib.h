#ifndef __FIFO_H__
#define __FIFO_H__

template <class T>
struct fifo_node
{
  struct fifo_node *next;
  T value;
};

template <class T>
class Fifo
{
  private:
    int m_size;

    fifo_node<T> *m_front;
    fifo_node<T> *m_back;

  public:
    Fifo();
    T dequeue(void);
    void enqueue(T value);
    int available();
};

#endif
