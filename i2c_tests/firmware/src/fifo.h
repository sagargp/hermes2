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
  if (m_front != NULL)
  {
    fifo_node<T> *temp = m_front;
    T value = temp->value;
    m_front = m_front->next;
    m_size--;
    delete temp;
    return value;
  }
  else
  {
    m_back = NULL;
    m_size = 0;
    return -1;
  }
}

template <class T>
int Fifo<T>::available()
{
  return m_size;
}
