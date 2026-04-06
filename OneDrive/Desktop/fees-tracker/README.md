# BursarPro - School Fee Management System

BursarPro is a lightweight, comprehensive school fee management web application designed for Ugandan schools. It enables educators and administrators to efficiently track student payments, calculate account balances, send automated SMS reminders, and generate detailed PDF reports.

## System Overview

BursarPro is built with a modern, scalable architecture:

- **Frontend**: React-based single-page application with responsive UI
- **Backend**: Django REST API providing robust business logic and data management
- **Database**: Supabase PostgreSQL for reliable, secure data storage
- **Real-time Features**: WebSocket support for live activity tracking and notifications

## Key Features

- **Payment Tracking**: Monitor individual and bulk student fee payments in real-time
- **Balance Calculations**: Automated computation of student account balances and outstanding fees
- **SMS Notifications**: Send automated payment reminders to parents and guardians
- **PDF Reports**: Generate comprehensive financial and administrative reports
- **Multi-Role Support**: Distinct dashboards and permissions for Headmasters, Finance Officers, Teachers, and Parents
- **Bulk Payment Processing**: Handle multiple payments efficiently
- **Activity Auditing**: Track all financial transactions and system activities
- **Search & Analytics**: Advanced search capabilities and analytics dashboard

## Project Structure

```
fees-tracker/
├── bursarpro/
│   ├── backend/          # Django REST API
│   │   ├── core/         # Django configuration
│   │   ├── finance/      # Core business logic & models
│   │   └── manage.py     # Django management script
│   └── frontend/         # React application
│       ├── src/          # React components & services
│       └── vite.config.js # Build configuration
├── requirements.txt      # Python dependencies
└── [Test & Setup Files]  # Integration tests and initialization scripts
```

## Getting Started

See [SETUP_GUIDE.md](SETUP_GUIDE.md) for detailed installation and configuration instructions.

## Documentation

- [Backend README](bursarpro/backend/README.md) - API documentation and backend architecture
- [Frontend README](bursarpro/frontend/README.md) - Frontend setup and components
- [Setup Guide](SETUP_GUIDE.md) - Installation and initial configuration
- [Troubleshooting](TROUBLESHOOTING.md) - Common issues and solutions
- [Pre-Launch Checklist](PRE-LAUNCH-CHECKLIST.md) - Final verification steps

## Database & Services

- PostgreSQL via Supabase for reliable data persistence
- Activity logging and audit trails for compliance
- WebSocket support for real-time notifications
- SMS integration for parent communication
- Reporting and analytics engine

## Development Status

Check [COMPLETE_STATUS.md](COMPLETE_STATUS.md) for the latest feature completion status and [REMAINING_TASKS.md](REMAINING_TASKS.md) for upcoming work.

---

**Version**: 1.0  
**Last Updated**: April 2026
