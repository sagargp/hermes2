#include <assert.h>
#include <iostream>
#include <cstddef>
#include "../src/fifo.h"

using namespace std;

int main()
{
  // make a new fifo
  Fifo<int> test_fifo;

  // make sure it knows that it's empty
  assert(test_fifo.available() == 0);

  // fill it with 0-9
  for (int i = 0; i < 10; i++)
    test_fifo.enqueue(i);

  // make sure it knows it's full of 10 things
  assert(test_fifo.available() == 10);

  // try removing (should remove the first element)
  assert(test_fifo.dequeue() == 0);

  // make sure the size has changed to reflect the removed element
  assert(test_fifo.available() == 9);

  // remove the rest of the elements in order, checking them each
  for (int i = 1; i < 10; i++)
    assert(test_fifo.dequeue() == i);

  // make sure it knows it's empty again
  assert(test_fifo.available() == 0);

  // make sure you can't get anything else out of it
  assert(test_fifo.dequeue() == -1);

  // put a ton of stuff in it
  for (int i = 0; i < 10000; i++)
    test_fifo.enqueue(i);
  assert(test_fifo.available() == 10000);

  // now empty it
  while (test_fifo.available())
    test_fifo.dequeue();
  assert(test_fifo.available() == 0);
  assert(test_fifo.dequeue() == -1);
}
