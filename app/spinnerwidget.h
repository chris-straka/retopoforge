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
#ifndef RETOPO_SPINNER_WIDGET_H
#define RETOPO_SPINNER_WIDGET_H
#include <QColor>
#include <QTimer>
#include <QWidget>

// Native replacement for the third-party QtWaitingSpinner widget: a
// QTimer-driven ring of fading arc segments painted with QPainter.
class SpinnerWidget : public QWidget {
    Q_OBJECT
public:
    explicit SpinnerWidget(QWidget* parent = nullptr);

public slots:
    void start();
    void stop();

public:
    void setColor(const QColor& color);
    void setInnerRadius(int radius);
    void setLineLength(int length);
    void setNumberOfLines(int lines);

    QColor color() const;
    int innerRadius() const;
    int lineLength() const;
    int numberOfLines() const;

    bool isSpinning() const;

protected:
    void paintEvent(QPaintEvent* event) override;

private slots:
    void advance();

private:
    void updateSize();
    void updateTimer();

    QColor m_color;
    int m_innerRadius;
    int m_lineLength;
    int m_numberOfLines;
    int m_head;
    bool m_spinning;
    QTimer* m_timer;
};

#endif
