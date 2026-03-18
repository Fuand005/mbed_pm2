#include "MedianFilter3.h"

MedianFilter3::MedianFilter3()
{
    init();
}

void MedianFilter3::init()
{
    m_idx = 0;
    reset(0.0f);
}

float MedianFilter3::reset(float val)
{
    // fill the ring buffer with 'val'
    for (int i = 0; i < 3; i++) {
        m_window[i] = val;
    }
    // the median is trivially 'val' at this point
    m_val = val;
    m_idx = 0;

    return m_val;
}

float MedianFilter3::reset()
{
    return reset(0.0f);
}

float MedianFilter3::apply(float inp)
{
    // store the new sample at the current index
    m_window[m_idx] = inp;

    // advance the ring buffer index
    m_idx = (m_idx + 1) % 3;

    // copy to a local array for sorting
    float arr[3];
    for (int i = 0; i < 3; i++) {
        arr[i] = m_window[i];
    }

    // simple bubble sort for 3 elements
    for (int i = 0; i < 2; i++) {
        for (int j = i + 1; j < 3; j++) {
            if (arr[i] > arr[j]) {
                float tmp = arr[i];
                arr[i]   = arr[j];
                arr[j]   = tmp;
            }
        }
    }

    // the middle element (index 1) is the median
    m_val = arr[1];
    return m_val;
}
