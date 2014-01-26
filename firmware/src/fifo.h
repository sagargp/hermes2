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

    fifo_node<T> *front;
    fifo_node<T> *back;

  public:
    fifo_node<T> *dequeue(void);
    void enqueue(T value);
    int available();
};

template <class T>
void Fifo<T>::enqueue(T value)
{
  fifo_node<T> *temp = new fifo_node<T>;
  temp->value = value;

  if (front == NULL)
  {
    front = temp;
    back = temp;
    m_size = 0;
  }
  else
  {
    back->next = temp;
    back = temp;
    m_size++;
  }
}

template <class T>
fifo_node<T> *Fifo<T>::dequeue()
{
  fifo_node<T> *temp = front;

  if (front != NULL)
  {
    front = front->next;
    m_size--;
  }
  else
  {
    back = NULL;
    m_size = 0;
  }
  return temp;
}

template <class T>
int Fifo<T>::available()
{
  return m_size;
}
