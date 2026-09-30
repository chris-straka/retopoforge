/*
 *  Copyright (c) 2026 retopoforge contributors. All rights reserved.
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */
#include "spinnerwidget.h"
#include <QPainter>
#include <QPaintEvent>
#include <QtMath>

namespace {

// Ring turns this many times per second at the head segment.
constexpr qreal revolutionsPerSecond = 1.5;
// Faintest trail segment opacity, 0..1.
constexpr qreal minimumTrailOpacity = 0.05;
// Gap between adjacent arc segments, in degrees.
constexpr qreal segmentGapDegrees = 8.0;

}

SpinnerWidget::SpinnerWidget(QWidget* parent)
    : QWidget(parent)
    , m_color(Qt::black)
    , m_innerRadius(10)
    , m_lineLength(10)
    , m_numberOfLines(12)
    , m_head(0)
    , m_spinning(false)
    , m_timer(new QTimer(this))
{
    connect(m_timer, &QTimer::timeout, this, &SpinnerWidget::advance);
    updateSize();
    updateTimer();
    hide();
}

void SpinnerWidget::start()
{
    m_spinning = true;
    show();
    if (!m_timer->isActive()) {
        m_timer->start();
        m_head = 0;
    }
}

void SpinnerWidget::stop()
{
    m_spinning = false;
    hide();
    if (m_timer->isActive()) {
        m_timer->stop();
        m_head = 0;
    }
}

void SpinnerWidget::setColor(const QColor& color)
{
    m_color = color;
    update();
}

void SpinnerWidget::setInnerRadius(int radius)
{
    m_innerRadius = radius;
    updateSize();
}

void SpinnerWidget::setLineLength(int length)
{
    m_lineLength = length;
    updateSize();
}

void SpinnerWidget::setNumberOfLines(int lines)
{
    if (lines < 1)
        lines = 1;
    m_numberOfLines = lines;
    m_head = 0;
    updateTimer();
    update();
}

QColor SpinnerWidget::color() const
{
    return m_color;
}

int SpinnerWidget::innerRadius() const
{
    return m_innerRadius;
}

int SpinnerWidget::lineLength() const
{
    return m_lineLength;
}

int SpinnerWidget::numberOfLines() const
{
    return m_numberOfLines;
}

bool SpinnerWidget::isSpinning() const
{
    return m_spinning;
}

void SpinnerWidget::advance()
{
    m_head = (m_head + 1) % m_numberOfLines;
    update();
}

void SpinnerWidget::updateSize()
{
    const int extent = 2 * (m_innerRadius + m_lineLength);
    setFixedSize(extent, extent);
}

void SpinnerWidget::updateTimer()
{
    m_timer->setInterval(qMax(1, qRound(1000.0 / (m_numberOfLines * revolutionsPerSecond))));
}

void SpinnerWidget::paintEvent(QPaintEvent*)
{
    if (parentWidget()) {
        const int x = (parentWidget()->width() - width()) / 2;
        const int y = (parentWidget()->height() - height()) / 2;
        if (pos() != QPoint(x, y))
            move(x, y);
    }

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);

    const qreal penWidth = qMax<qreal>(2.0, m_lineLength / 3.0);
    const qreal radius = m_innerRadius + penWidth / 2.0;
    const QPointF center(width() / 2.0, height() / 2.0);
    const QRectF ring(center.x() - radius, center.y() - radius, 2.0 * radius, 2.0 * radius);
    const qreal spanPerSegment = 360.0 / m_numberOfLines;
    const qreal arcSpan = qMax<qreal>(1.0, spanPerSegment - segmentGapDegrees);

    for (int i = 0; i < m_numberOfLines; ++i) {
        const int distance = (m_head - i + m_numberOfLines) % m_numberOfLines;
        const qreal t = m_numberOfLines > 1
            ? 1.0 - static_cast<qreal>(distance) / static_cast<qreal>(m_numberOfLines - 1)
            : 1.0;
        QColor color = m_color;
        color.setAlphaF(minimumTrailOpacity + (1.0 - minimumTrailOpacity) * t);
        QPen pen(color, penWidth, Qt::SolidLine, Qt::RoundCap);
        painter.setPen(pen);
        const qreal startAngle = i * spanPerSegment * 16.0;
        painter.drawArc(ring, qRound(startAngle), qRound(arcSpan * 16.0));
    }
}
