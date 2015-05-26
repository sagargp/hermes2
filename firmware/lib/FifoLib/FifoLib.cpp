#include <Arduino.h>
#include "FifoLib.h"

template <class T>
Fifo<T>::Fifo() :
  m_front(NULL),
  m_back(NULL),
  m_size(0)
{
}

template <class T>
void Fifo<T>::enqueue(T value)
{
  fifo_node<T> *temp = new fifo_node<T>;
  temp->value = value;

  if (m_front == NULL)
  {
    m_front = temp;
    m_back = temp;
    m_size = 1;
  }
  else
  {
    m_back->next = temp;
    m_back = temp;
    m_size++;
  }
}

template <class T>
T Fifo<T>::dequeue()
{
  fifo_node<T> *temp = m_front;

  if (m_front != NULL)
  {
    m_front = m_front->next;
    m_size--;
  }
  else
  {
    m_back = NULL;
    m_size = 0;
  }

  if (temp != NULL)
  {
    T value = temp->value;
    delete temp;
    return value;
  }
  else
  {
    return 0;
  }
}

template <class T>
int Fifo<T>::available()
{
  return m_size;
}
