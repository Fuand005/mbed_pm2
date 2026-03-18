#include "AvgFilter.h"

AvgFilter::AvgFilter(uint8_t N)
{
    init(N);
}

AvgFilter::~AvgFilter() {
    if (m_ring_buffer) {
        free(m_ring_buffer);
        m_ring_buffer = nullptr;
    }
}

void AvgFilter::init(uint8_t N)
{
    m_N = N;
    // allocate space for the ring buffer (each element is a float)
    m_ring_buffer = (float*)malloc(m_N * sizeof(float));
    // reset the filter (fills ring buffer with zeros by default)
    reset();
}

float AvgFilter::reset(float val)
{
    // set the filter's rolling sum to 'val'
    m_val = val;

    // each ring buffer element = val / N => total sums up to 'val'
    const float scaled_val = val / (float)m_N;

    // start writing at index 0
    m_idx = 0;

    // fill the ring buffer with scaled_val
    for (uint8_t i = 0; i < m_N; i++)
        m_ring_buffer[i] = scaled_val;

    return m_val;
}

float AvgFilter::reset()
{
    return reset(0.0f);
}

float AvgFilter::apply(float inp)
{
    // remove the old value from the rolling sum
    // (the ring buffer stores scaled values, so subtract that)
    m_val -= m_ring_buffer[m_idx];

    // compute scaled version of the new input and add it to the rolling sum
    const float scaled_inp = inp / (float)m_N;
    m_val += scaled_inp;

    // store the scaled value in the ring buffer at m_idx
    m_ring_buffer[m_idx] = scaled_inp;

    // move to the next position in the ring buffer (wrap around if needed)
    m_idx++;
    if (m_idx == m_N)
        m_idx = 0;

    // return the new rolling sum (which is effectively the average)
    return m_val;
}
